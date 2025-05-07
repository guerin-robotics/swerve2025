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

public class Constants {
    public static class Vision {

        public static final String kCameraName = "HoundEye04";
        // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
        public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

        // The layout of the AprilTags on the field
        public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The standard deviations of our vision estimated poses, which affect correction rate
        // (Fake values. Experiment and determine estimation noise on an actual robot.)
        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static final double masterSpeedMultiplier = 1;   // For troubleshooting/testing
    public static final double masterVoltageMultiplier = 1;

    public static final class Joystick {
        public static final int Function1 = 10;
        public static final int Function2 = 9;
        public static final int strafeLeft = 3;
        public static final int strafeRight = 4;
        public static final int servoControl = 8;
    }

    public static final class buttonPanel {
        public static final class lift {
            public static final int L1 = 3;
            public static final int L2 = 2;
            public static final int L3 = 4;
            public static final int L4 = 8;
        }
        public static final class coral {
            public static final int In = 1;
            public static final int Out = 7;
        }
        public static final class algae {
            public static final int Lower = 5;
            public static final int Upper = 9;
            public static final int Retract = 10;
        }
    }

    public static final class XboxController {
        public static final class bumper {
            public static final int Left = 5;
            public static final int Right = 6;
        }

        public static final class button {
            public static final int A = 1;
            public static final int B = 2;
            public static final int X = 3;
            public static final int Y = 4;
            public static final int Start = 8;
            public static final int Window = 7;
        }

        public static final class axis {
            public static final int LeftXAxis = 0;
            public static final int LeftYAxis = 1;
            public static final int RightXAxis = 4;
            public static final int RightYAxis = 5;
            public static final int LeftTrigger = 2;
            public static final int RightTrigger = 3;
        }

        public static final class dpad {
            public static final int Up = 0;
            public static final int Right = 90;
            public static final int Down = 180;
            public static final int Left = 270;
        }
}

    public static final class elevator {
        public static final int LiftLeft = 10;
        public static final int LiftRight = 11;

        public static final class level {
            public static final double L1 = 0.05;
            public static final double L2 = 17.5;
            public static final double L3 = 38.0;
            public static double L4 = 74.0;
        }

        public static final class algaeLevel {
            public static final double L2 = 30.0;
            public static final double L3 = 49.0;
        }
    }

    public static final class effector {
        public static final int EffectorLeft = 12;
        public static final int EffectorRight = 13;
        public static final double defaultVelocity = 15;
    }

    public static final class hang {
        public static final int hangMotor = 14;
    }
    
    public static void setL4() {
        System.out.println("Driver side:" + DriverStationJNI.getAllianceStation().toString());
        var driverStation = DriverStationJNI.getAllianceStation().toString();
        if (driverStation == ("Blue1")) {
            elevator.level.L4 = 72.0;
            System.out.println("Setting blue side L4");
        }
        else if (driverStation == ("Blue2")) {
            elevator.level.L4 = 72.0;
            System.out.println("Setting blue side L4");
        }
        else if (driverStation == ("Blue3")) {
            elevator.level.L4 = 72.0;
            System.out.println("Setting blue side L4");
        }
        else {
            elevator.level.L4 = 72.5; //74.0 for good wheels
            System.out.println("Setting red side L4");
        }
    }
}
