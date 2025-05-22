package frc.robot;

import static frc.robot.Constants.Vision.*;
import static frc.robot.generated.TunerConstants.DrivetrainConstants;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Logger.Level;
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
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTableValue;
import frc.robot.RobotContainer;
import com.ctre.phoenix6.Utils;

public class Vision {
    private final PhotonCamera camera0 = new PhotonCamera(kCameraName + "04");
    // private final PhotonCamera camera1 = new PhotonCamera(kCameraName + "5");

    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private final EstimateConsumer estConsumer;

    /**
     * @param estConsumer Lamba that will accept a pose estimate and pass it to your
     *                    desired {@link
     *                    edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}
     */

    public Vision(EstimateConsumer estConsumer) {
        this.estConsumer = estConsumer;

        // Select the PhotonVision pipeline (0-based index) to use for processing

        // camera1.setPipelineIndex(0);

        camera0.setPipelineIndex(0);

        photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // Initialize NetworkTables entries for external monitoring
    }

    public void periodic() {
        // Always fetch the latest pipeline result to ensure we see current detections
        processFrame(camera0, 1);
    }

    private void processFrame(PhotonCamera cam, int camId) {
        var results = cam.getAllUnreadResults();
        Logger.debug("Cam {} pipeline results [{}]", camId, results.size());
        if (results.isEmpty()) {
            Logger.debug("Cam {} pipeline no results", camId);
            return;
        }

        var last = results.get(results.size() - 1);
        var maybeEst = photonEstimator.update(last);
        if (maybeEst.isEmpty()) {
            Logger.debug("Cam {} no targets detected this cycle", camId);
            return;
        }

        var visionEst = maybeEst.get();
        var visionPose = visionEst.estimatedPose.toPose2d();
        var targets = last.getTargets();

        // Heuristic: pick σ based on # tags & average tag‐to‐vision distance
        Matrix<N3, N1> stdDevs = kSingleTagStdDevs;
        int numTags = 0;
        double avgTagDist = 0;
        for (var tgt : targets) {
            var tagPose = photonEstimator.getFieldTags().getTagPose(
                    tgt.getFiducialId());
            if (tagPose.isEmpty()) {
                Logger.warn("tagPose is empty");
                continue;
            }
            numTags++;
            avgTagDist += tagPose
                    .get()
                    .toPose2d()
                    .getTranslation()
                    .getDistance(visionPose.getTranslation());
            ;
        }
        if (numTags > 0) {
            avgTagDist /= numTags;
            if (numTags > 1) {
                stdDevs = kMultiTagStdDevs;
            }
            if (numTags == 1 && avgTagDist > 4.0) {
                stdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else {
                stdDevs = stdDevs.times(1 + (avgTagDist * avgTagDist / 30.0));
            }
        } else {
            // No tags visible. Default to single-tag std devs
            Logger.debug("no tags visible");
            curStdDevs = kSingleTagStdDevs;
        }

        // Now check how far visionPose is from our current odometry pose:
        double distanceToRobot = frc.robot.RobotContainer.drivetrain.getPose().getTranslation()
                .getDistance(visionEst.estimatedPose.toPose2d().getTranslation());
        if (distanceToRobot < .5) {
            Logger.debug("Cam {} fusing vision ({} tags, avgTagDist={:.2f}, robotDist={:.2f})",
                    camId, numTags, avgTagDist, distanceToRobot);

            RobotContainer.drivetrain.addVisionMeasurement(
                    visionPose,
                    Utils.fpgaToCurrentTime(visionEst.timestampSeconds),
                    stdDevs);
        } else {
            Logger.debug("Cam {} skipping fusion; {:.2f} m away", camId, distanceToRobot);
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator
     * SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        Logger.debug("current std dev [{}]", curStdDevs);
        return curStdDevs;
    }

    @FunctionalInterface
    public static interface EstimateConsumer {
        public void accept(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs);
    }
}