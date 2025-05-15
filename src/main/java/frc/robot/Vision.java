/*
 * MIT License
 *
 * Copyright (c) PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

 package frc.robot;

 import static frc.robot.Constants.Vision.*;
 
 import edu.wpi.first.math.Matrix;
 import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.Timer;
 import java.util.List;
 import java.util.Optional;
 import java.util.stream.Collectors;
 import org.photonvision.EstimatedRobotPose;
 import org.photonvision.PhotonCamera;
 import org.photonvision.PhotonPoseEstimator;
 import org.photonvision.PhotonPoseEstimator.PoseStrategy;
 import org.photonvision.simulation.PhotonCameraSim;
 import org.photonvision.simulation.SimCameraProperties;
 import org.photonvision.simulation.VisionSystemSim;
 import org.photonvision.targeting.PhotonTrackedTarget;
 import edu.wpi.first.networktables.NetworkTable;
 import edu.wpi.first.networktables.NetworkTableEntry;
 import edu.wpi.first.networktables.NetworkTableInstance;

 public class Vision {
     private final PhotonCamera camera;
     private final PhotonPoseEstimator photonEstimator;
     private Matrix<N3, N1> curStdDevs;
     private final EstimateConsumer estConsumer;
 
     // Simulation
     private PhotonCameraSim cameraSim;
     private VisionSystemSim visionSim;

     // NetworkTables entries for publishing vision pose
     private final NetworkTable visionTable;
     private final NetworkTableEntry visionX;
     private final NetworkTableEntry visionY;
     private final NetworkTableEntry visionHeading;
 
     /**
      * @param estConsumer Lamba that will accept a pose estimate and pass it to your desired {@link
      *     edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
      */
     public Vision(EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;
        camera = new PhotonCamera(kCameraName);
        // Select the PhotonVision pipeline (0-based index) to use for processing
        camera.setPipelineIndex(0);
 
         photonEstimator =
                 new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
         photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
 
         // ----- Simulation
         if (Robot.isSimulation()) {
             // Create the vision system simulation which handles cameras and targets on the field.
             visionSim = new VisionSystemSim("main");
             // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
             visionSim.addAprilTags(kTagLayout);
             // Create simulated camera properties. These can be set to mimic your actual camera.
             var cameraProp = new SimCameraProperties();
             cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
             cameraProp.setCalibError(0.35, 0.10);
             cameraProp.setFPS(15);
             cameraProp.setAvgLatencyMs(50);
             cameraProp.setLatencyStdDevMs(15);
             // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
             // targets.
             cameraSim = new PhotonCameraSim(camera, cameraProp);
             // Add the simulated camera to view the targets on this simulated field.
             visionSim.addCamera(cameraSim, kRobotToCam);
 
             cameraSim.enableDrawWireframe(true);
         }
         // Initialize NetworkTables entries for external monitoring
         visionTable = NetworkTableInstance.getDefault().getTable("Vision");
         visionX = visionTable.getEntry("X");
         visionY = visionTable.getEntry("Y");
         visionHeading = visionTable.getEntry("Heading");
     }
 
     public void periodic() {
         // Always fetch the latest pipeline result to ensure we see current detections
         var result = camera.getLatestResult();
         Optional<EstimatedRobotPose> visionEst = photonEstimator.update(result);
         updateEstimationStdDevs(visionEst, result.getTargets());

         if (Robot.isSimulation()) {
             visionEst.ifPresentOrElse(
                 est -> getSimDebugField().getObject("VisionEstimation").setPose(est.estimatedPose.toPose2d()),
                 ()  -> getSimDebugField().getObject("VisionEstimation").setPoses()
             );
         }

         // Log detections or misses
        if (visionEst.isPresent()) {
            var est = visionEst.get();
            System.out.println("PhotonVision: detected " + result.getTargets().size()
                + " tags, pose=" + est.estimatedPose.toPose2d());
            // Use FPGA timestamp directly for filter updates
            estConsumer.accept(est.estimatedPose.toPose2d(), Timer.getFPGATimestamp(), getEstimationStdDevs());
        } else {
            System.out.println("PhotonVision: no targets detected this cycle");
        }

         // Publish to NetworkTables
         visionEst.ifPresent(est -> {
             Pose2d p = est.estimatedPose.toPose2d();
             visionX.setDouble(p.getX());
             visionY.setDouble(p.getY());
             visionHeading.setDouble(p.getRotation().getDegrees());
         });
     }
 
     /**
      * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
      * deviations based on number of tags, estimation strategy, and distance from the tags.
      *
      * @param estimatedPose The estimated pose to guess standard deviations for.
      * @param targets All targets in this camera frame
      */
     private void updateEstimationStdDevs(
             Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
         if (estimatedPose.isEmpty()) {
             // No pose input. Default to single-tag std devs
             curStdDevs = kSingleTagStdDevs;
             System.out.println("PhotonVision: no pose estimate");
 
         } else {
             // Pose present. Start running Heuristic
             var estStdDevs = kSingleTagStdDevs;
             int numTags = 0;
             double avgDist = 0;
             System.out.println("PhotonVision: pose estimate=" + estimatedPose.get().estimatedPose.toPose2d());
             // Precalculation - see how many tags we found, and calculate an average-distance metric
             for (var tgt : targets) {
                 var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                 if (tagPose.isEmpty()) continue;
                 numTags++;
                 avgDist +=
                         tagPose
                                 .get()
                                 .toPose2d()
                                 .getTranslation()
                                 .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
             }
 
             if (numTags == 0) {
                 // No tags visible. Default to single-tag std devs
                 System.out.println("PhotonVision: no tags visible");
                 curStdDevs = kSingleTagStdDevs;
             } else {
                 // One or more tags visible, run the full heuristic.
                 System.out.println(
                         "PhotonVision: " + numTags + " tags visible, avg distance=" + (avgDist / numTags));
                 avgDist /= numTags;
                 // Decrease std devs if multiple targets are visible
                 if (numTags > 1) estStdDevs = kMultiTagStdDevs;
                 // Increase std devs based on (average) distance
                 if (numTags == 1 && avgDist > 4)
                     estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                 else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                 curStdDevs = estStdDevs;
             }
         }
     }
 
     /**
      * Returns the latest standard deviations of the estimated pose from {@link
      * #getEstimatedGlobalPose()}, for use with {@link
      * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
      * only be used when there are targets visible.
      */
     public Matrix<N3, N1> getEstimationStdDevs() {
        System.out.println(curStdDevs);
        return curStdDevs;
     }
 
     // ----- Simulation
 
     public void simulationPeriodic(Pose2d robotSimPose) {
         visionSim.update(robotSimPose);
     }
 
     /** Reset pose history of the robot in the vision system simulation. */
     public void resetSimPose(Pose2d pose) {
         if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
     }
 
     /**
      * Returns the latest raw vision-based pose estimate (without odometry fusion).
      */
     public Optional<Pose2d> getLatestRawVisionPose() {
         var result = camera.getLatestResult();
         var est = photonEstimator.update(result);
         return est.map(e -> e.estimatedPose.toPose2d());
     }
 
     /** A Field2d for visualizing our robot and objects on the field. */
     public Field2d getSimDebugField() {
         if (!Robot.isSimulation()) return null;
         return visionSim.getDebugField();
     }
 
     @FunctionalInterface
     public static interface EstimateConsumer {
         public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
     }
 }