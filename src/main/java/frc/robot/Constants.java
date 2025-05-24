package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import static frc.robot.Constants.Vision.*;

import java.util.Optional;
import edu.wpi.first.math.geometry.Pose2d;

public class Constants {
    // Right Camera
    public static final String CameraName1 = "EagleEye01";

    // Left Camera
    public static final String CameraName2 = "EagleEye02";


    // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.

    // Right Cam
    public static final Transform3d RobotToCam1 = new Transform3d(new Translation3d(0.2786892, -0.2726416, 0.1499719),
            new Rotation3d(0, -0.3490659, 0.1745329));

    // Left Cam
    public static final Transform3d RobotToCam2 = new Transform3d(new Translation3d(0.2764772, 0.2724549, 0.1499719),
            new Rotation3d(0, -0.3490659, -0.1745329));

    public static final class Joystick {
        public static final int strafeLeft = 3;
        public static final int strafeRight = 4;
        public static final int Button1 = 10;
    }

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect
    // correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(
            0.25, // σₓ: odometry may drift ±10 cm
            0.25, // σᵧ: same sideways
            Math.toRadians(.05) // σθ: roughly ±5° heading error
    );
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(
            0.18, // σₓ: vision ±0.5 cm
            0.18, // σᵧ: ±0.5 cm
            Math.toRadians(.025) // σθ: ±2°
    );
    public static final Matrix<N3, N1> kOdometryStdDevs = VecBuilder.fill(
            0.075, // 2 cm
            0.075, // 2 cm
            Math.toRadians(.6) // 2°
    );
    public static final double masterSpeedMultiplier = 1; // For troubleshooting/testing
    public static final double masterVoltageMultiplier = 1;

    public static class Pathfinding {
        // max translation m/s
        public static final double MaxSpeed = 4;
        // max accel m/s²
        public static final double MaxAccel = 4;
        // max rot deg/s
        public static final double MaxRotSpeed = 600;
        // max rot accel deg/s²
        public static final double MaxRotAccel = 600;

    }

    /**
     * Vision-specific constants grouped for NetworkTables and PhotonVision use.
     */
    public static class Vision {
        public static final String CameraName1 = Constants.CameraName1;
        public static final Transform3d RobotToCam1 = Constants.RobotToCam1;
        public static final String CameraName2 = Constants.CameraName2;
        public static final Transform3d RobotToCam2 = Constants.RobotToCam2;
        public static final AprilTagFieldLayout kTagLayout = Constants.kTagLayout;
        public static final Matrix<N3, N1> kSingleTagStdDevs = Constants.kSingleTagStdDevs;
        public static final Matrix<N3, N1> kMultiTagStdDevs = Constants.kMultiTagStdDevs;
        public static final Matrix<N3, N1> kOdometryStdDevs = Constants.kOdometryStdDevs;

        /** Odometry update rate in Hz for the SwerveDrivePoseEstimator */
        public static final double kOdometryUpdateHz = 250.0;

        /** PhotonVision pipeline index to use (0-based) */
        public static final int kPipelineIndex = 0;

    }
}
