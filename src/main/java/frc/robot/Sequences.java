package frc.robot;

import frc.robot.subsystems.*;

public class Sequences {
    public static void removeL2Algae() {
        Elevator.toPosition(Constants.elevator.algaeLevel.L2);
        Effector.algaeEffectorUp();
        Elevator.toPosition(Constants.elevator.algaeLevel.L2 - 3);
    }

    public static void removeL3Algae() {
        Elevator.toPosition(Constants.elevator.algaeLevel.L3);
        Effector.algaeEffectorUp();
        Elevator.toPosition(Constants.elevator.algaeLevel.L3 - 3);
    }

    public static void scoreL1Coral() {
        Elevator.toPosition(Constants.elevator.level.L1);
        Effector.asymmetricalOuttake(null, null);
    }
}
