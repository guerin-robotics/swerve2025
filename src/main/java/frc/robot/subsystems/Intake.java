package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Intake extends SubsystemBase {

    private static Spark intakeActuator = new Spark(6);
    private static TalonFX intakeRight = new TalonFX(Constants.intakeMotors.intakeRightID);
    private static TalonFX intakeLeft = new TalonFX(Constants.intakeMotors.intakeLeftID);

    private final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

    public static void intakeDrop() {
        if (Timer.getMatchTime() < 30) {
            intakeActuator.set(-1);
        }
    }

    public static void intakeReset() {
        intakeActuator.set(1);
    }

    public static void intake() {
        intakeRight.setControl(new Follower(intakeLeft.getDeviceID(), false));
        intakeRight.setControl(m_velocityVoltage.withVelocity(10));
    }
}
