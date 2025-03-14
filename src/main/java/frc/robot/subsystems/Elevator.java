package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import frc.robot.Constants;
import frc.robot.Constants.elevator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Robot;

public class Elevator extends SubsystemBase{
    public static TalonFX liftLeft;
    public static TalonFX liftRight;
    public static DigitalInput bottomlimitSwitch = new DigitalInput(0);
    public static DigitalInput toplimitSwitch = new DigitalInput(1);
    // private static LaserCan elevatorBottom = new LaserCan(0);
    private static LaserCan elevatorTop = new LaserCan(1);

    private final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final static MotionMagicVoltage motionControl = new MotionMagicVoltage(0).withSlot(0);
                
    public Elevator() {
        System.out.println("Creating new motor with ID " + Constants.elevator.LiftLeft);

        liftLeft = new TalonFX(Constants.elevator.LiftLeft);
        liftRight = new TalonFX(Constants.elevator.LiftRight);
        liftLeft.setNeutralMode(NeutralModeValue.Brake);
        liftRight.setControl(new Follower(liftLeft.getDeviceID(), true));

        configureElevator();
    }

    // public static double calculateSpeed(double inputSpeed, double elevatorPosition) {
    //     if ((elevatorTop.getMeasurement().distance_mm <= 40) && (inputSpeed < 0)) {
    //         return 0.0;
    //     } else if ((elevatorPosition > 60) && (inputSpeed < 0)) {
    //         return inputSpeed * -(75 - elevatorPosition) * 10;
    //     } else if ((elevatorPosition < 9) && (inputSpeed > 0)) {
    //         return inputSpeed * -elevatorPosition * 10;
    //     } else {
    //         return inputSpeed * -100;
    //     }
    // }
                    
    public static void toPosition(double rotations) {
        System.out.println("Going to " + rotations);
        if (toplimitSwitch.get()) {
            System.out.println(motionControl.withPosition(rotations));
            liftLeft.setControl(motionControl.withPosition(rotations));
            // var status = liftLeft.setControl(m_velocityVoltage.withVelocity(-10));

            // liftLeft.set(5);
            System.out.println(liftLeft.getPosition() + " rotations reached.");
        } else{
            liftLeft.set(0);
            liftLeft.setControl(m_velocityVoltage.withVelocity(0));
        }
        return;
    }
    
    public static void toBottom() {
        while (bottomlimitSwitch.get()) {
            // liftLeft.setControl(motionControl.withPosition(0));
            liftLeft.setControl(m_velocityVoltage.withVelocity(10));
        }
        liftLeft.setControl(m_velocityVoltage.withVelocity(0));
    }
    
    public static void manualControl(double velocity) {
        double desiredRotationsPerSecond;
        double liftPosition = liftLeft.getPosition().getValueAsDouble();
        // liftLeft.setControl(motionControl.withFeedForward(-controller.getRawAxis(1)));
        System.out.println("Joystick at: " + velocity);
        // if (velocity == 0) {
        //     liftLeft.setControl(m_velocityVoltage.withVelocity(0));
        // }
        // else {
        //     liftLeft.setControl(m_velocityVoltage.withVelocity(-velocity * 5));
        // }
        if ((elevatorTop.getMeasurement().distance_mm <= 40) && (velocity < 0)) {
            desiredRotationsPerSecond = 0;
        } else if ((liftPosition > 60) && (velocity < 0)) {
            desiredRotationsPerSecond = velocity * -(75 - liftPosition) * 10;
        } else if ((liftPosition < 9) && (velocity > 0)) {
            desiredRotationsPerSecond = velocity * -liftPosition * 10;
        } else {
            desiredRotationsPerSecond = velocity * -100;
        }
        liftLeft.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
        // liftLeft.set(controller.getRawAxis(1));
    }

    public static void configureElevator() {
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        var limitConfigs = elevatorConfig.CurrentLimits;

        limitConfigs.SupplyCurrentLimit = 60;
        limitConfigs.StatorCurrentLimit = 120;
        limitConfigs.StatorCurrentLimitEnable = true;

        elevatorConfig.Voltage.withPeakForwardVoltage(Volts.of(12)).withPeakReverseVoltage(Volts.of(-12));

        MotionMagicConfigs motionConfig = elevatorConfig.MotionMagic;
        motionConfig.withMotionMagicCruiseVelocity(RotationsPerSecond.of(100)).withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(500));
        // PID CONSTANTS
        elevatorConfig.Slot0.kP = 1.5; //3
        elevatorConfig.Slot0.kI = 0.00; //0.03
        elevatorConfig.Slot0.kD = 0.01; //0.01
        elevatorConfig.Slot0.kV = 0.13; //0.13
        elevatorConfig.Slot0.kG = 0.22; //0.22
        elevatorConfig.Slot0.kS = 0;
        elevatorConfig.Slot0.kA = 0;

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = liftLeft.getConfigurator().apply(elevatorConfig);
            System.out.println("Elevator configs applied successfully");
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        System.out.println("Left elevator: " + liftLeft.getDeviceID());
        System.out.println("Right elevator: " + liftRight.getDeviceID());

    }

    public static double getPosition() {
        return liftLeft.getPosition().getValueAsDouble();
    }
}
