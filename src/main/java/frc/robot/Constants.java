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

    public static final String kCameraName = "HoundEye04";
    // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    public static final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.36, 0.0064, 0.15),
            new Rotation3d(0, 0, 0));

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(
            100, // σₓ: odometry may drift ±10 cm
            100, // σᵧ: same sideways
            Math.toRadians(5) // σθ: roughly ±5° heading error
    );
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(
            0.00005, // σₓ: vision ±0.5 cm
            0.00005, // σᵧ: ±0.5 cm
            Math.toRadians(2) // σθ: ±2°
    );

    public static final double masterSpeedMultiplier = 1; // For troubleshooting/testing
    public static final double masterVoltageMultiplier = 1;

    public static final class Joystick {
        public static final int Function1 = 10;
        public static final int Function2 = 9;
        public static final int strafeLeft = 3;
        public static final int strafeRight = 4;
        public static final int servoControl = 8;
    }

    /**
     * Vision-specific constants grouped for NetworkTables and PhotonVision use.
     */
    public static class Vision {
        public static final String kCameraName = Constants.kCameraName;
        public static final Transform3d kRobotToCam = Constants.kRobotToCam;
        public static final AprilTagFieldLayout kTagLayout = Constants.kTagLayout;
        public static final Matrix<N3, N1> kSingleTagStdDevs = Constants.kSingleTagStdDevs;
        public static final Matrix<N3, N1> kMultiTagStdDevs = Constants.kMultiTagStdDevs;

        /** Odometry update rate in Hz for the SwerveDrivePoseEstimator */
        public static final double kOdometryUpdateHz = 250.0;

        /** PhotonVision pipeline index to use (0-based) */
        public static final int kPipelineIndex = 0;

    }
}
