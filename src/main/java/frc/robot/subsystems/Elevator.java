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
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import frc.robot.Constants;
import frc.robot.Constants.elevator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Elevator extends SubsystemBase{
    private static TalonFX liftLeft = new TalonFX(Constants.elevator.LiftLeft);
    private static TalonFX liftRight = new TalonFX(Constants.elevator.LiftRight);
    private static DigitalInput bottomlimitSwitch = new DigitalInput(0);
    private static DigitalInput toplimitSwitch = new DigitalInput(1);
    private static LaserCan elevatorBottom = new LaserCan(0);
    private static LaserCan elevatorTop = new LaserCan(1);

    private final static MotionMagicVoltage motionControl = new MotionMagicVoltage(0);
            
    public Elevator() {
        liftLeft.setNeutralMode(NeutralModeValue.Brake);
        liftRight.setControl(new Follower(liftLeft.getDeviceID(), true));

        configureElevator();
    }
                
    public void configureElevator() {
        TalonFXConfiguration elevatorConfig = new TalonFXConfiguration();
        var limitConfigs = new CurrentLimitsConfigs();

        limitConfigs.SupplyCurrentLimit = 60;
        limitConfigs.StatorCurrentLimit = 120;
        limitConfigs.StatorCurrentLimitEnable = true;

        

        elevatorConfig.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = liftLeft.getConfigurator().apply(elevatorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }

        MotionMagicConfigs motionConfig = elevatorConfig.MotionMagic;
        motionConfig.withMotionMagicCruiseVelocity(RotationsPerSecond.of(10)).withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(20));
        // PID CONSTANTS
        elevatorConfig.Slot0.kP = 0.12;
        elevatorConfig.Slot0.kI = 0;
        elevatorConfig.Slot0.kD = 0.01;
        elevatorConfig.Slot0.kV = 0.13;
        elevatorConfig.Slot0.kG = 0.2;
        elevatorConfig.Slot0.kS = 0;
        elevatorConfig.Slot0.kA = 0;
    }
        
    public static double calculateSpeed(double inputSpeed, double elevatorPosition) {
        if ((elevatorTop.getMeasurement().distance_mm <= 40) && (inputSpeed < 0)) {
            return 0.0;
        } else if ((elevatorPosition > 60) && (inputSpeed < 0)) {
            return inputSpeed * -(75 - elevatorPosition) * 10;
        } else if ((elevatorPosition < 9) && (inputSpeed > 0)) {
            return inputSpeed * -elevatorPosition * 10;
        } else {
            return inputSpeed * -100;
        }
    }
                    
    public void toPosition(double rotations) {
        System.out.println("Going to " + rotations);
        if (toplimitSwitch.get()) {
        System.out.println(rotations + " rotations reached.");
        liftLeft.setControl(motionControl.withPosition(rotations));
        }
    }
    
    public void toBottom() {
        if (bottomlimitSwitch.get()) {
            liftLeft.setControl(motionControl.withPosition(0));
        }
    }
    
    public void manualControl(CommandJoystick controller) {
        liftLeft.setControl(motionControl.withFeedForward(-controller.getRawAxis(1)));
    }
}
