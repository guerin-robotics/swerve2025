package frc.robot;

public class Constants {
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
        }

        public static final class axis {
            public static final int LeftXAxis = 0;
            public static final int LeftYAxis = 1;
            public static final int RightXAxis = 4;
            public static final int RightYAxis = 5;
            public static final int LeftTrigger = 2;
            public static final int RightTrigger = 3;
    }
}

    public static final class elevator {
        public static final int LiftLeft = 10;
        public static final int LiftRight = 11;

        public static final class level {
            public static final double L1 = 0.2;
            public static final double L2 = 18.0;
            public static final double L3 = 38.0;
            public static final double L4 = 73.0;
        }

        public static final class algaeLevel {
            public static final double L2 = 34.0;
            public static final double L3 = 50.0;
        }
    }

    public static final class effector {
        public static final int EffectorLeft = 12;
        public static final int EffectorRight = 13;
        public static final double defaultVelocity = 30;
    }
}
