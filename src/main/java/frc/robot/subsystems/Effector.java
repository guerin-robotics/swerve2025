package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import au.grapplerobotics.LaserCan;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.*;

public class Effector extends SubsystemBase {
    private static LaserCan intakeSensor;

    private static TalonFX effectorLeft;
    private static TalonFX effectorRight;

    private static SparkMax algaeMotor = new SparkMax(1, MotorType.kBrushed);

    private static Timer effectorTimer = new Timer();

    private final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

    private volatile static boolean isAlgaeOut = false;
    
    public Effector() {
        intakeSensor = new LaserCan(2);

        effectorLeft = new TalonFX(Constants.effector.EffectorLeft);
        effectorRight = new TalonFX(Constants.effector.EffectorRight);

        effectorLeft.setNeutralMode(NeutralModeValue.Coast);
        effectorRight.setNeutralMode(NeutralModeValue.Coast);
        // effectorRight.setControl(new Follower(effectorLeft.getDeviceID(), true));
        ConfigureEffector();
    }

    public void ConfigureEffector() {
        TalonFXConfiguration effectorConfig = new TalonFXConfiguration();
        var limitConfigs = new CurrentLimitsConfigs();

        effectorTimer = new Timer();

        effectorConfig.Slot0.kS = 0; // Static friction
        effectorConfig.Slot0.kV = 0.12; // 0.12 for Kraken X60
        effectorConfig.Slot0.kP = 0.1; // Rotational error per second
        effectorConfig.Slot0.kI = 0; // Integrated error
        effectorConfig.Slot0.kD = 0; // Error derivative

        effectorConfig.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = effectorLeft.getConfigurator().apply(effectorConfig);
            effectorRight.getConfigurator().apply(effectorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    public static void intakeUntilDetected() {
        while (intakeSensor.getMeasurement().distance_mm > 10) {
            effectorLeft.set(50);
            effectorRight.set(-50);
        }
        effectorTimer.start();
        while (effectorTimer.get() < 0.5) {
            effectorLeft.set(50);
            effectorRight.set(-50);
        }
        effectorLeft.set(0);
        effectorRight.set(0);
        effectorTimer.stop();
        effectorTimer.reset();
    }

    public static void outtakeUntilDetected() {
        while (intakeSensor.getMeasurement().distance_mm < 10) {
            effectorLeft.set(30);
            effectorRight.set(-30);
        }
        effectorTimer.start();
        while (effectorTimer.get() < 1.0) {
            effectorLeft.set(30);
            effectorRight.set(-30);
        } 
        effectorLeft.set(0);
        effectorRight.set(0);
        effectorTimer.stop();
        effectorTimer.reset();
    }

    public static void symmetricalOuttake(Double velocity) {
        double motorSpeed;
        if (velocity != null) {
            motorSpeed = velocity;
        }
        else {
            motorSpeed = Constants.effector.defaultVelocity;
        }
        effectorTimer.start();
        while (effectorTimer.get() < 2) {
            effectorLeft.set(motorSpeed);
            effectorRight.set(-motorSpeed);
        }
        effectorLeft.set(0);
        effectorRight.set(0);
        effectorTimer.stop();
        effectorTimer.reset();
    }

    public static void asymmetricalOuttake(Double velocityLeft, Double velocityRight) {
        double motorSpeedL;
        double motorSpeedR;
        if (velocityLeft != null) {
            motorSpeedL = velocityLeft;
        }
        else {
            motorSpeedL = Constants.effector.defaultVelocity * 2.5;
        }
        if (velocityRight != null) {
            motorSpeedR = velocityRight;
        }
        else {
            motorSpeedR = 0;
        }
        effectorTimer.start();
        while (effectorTimer.get() < 1) {
            effectorLeft.set(motorSpeedL);
            effectorRight.set(-motorSpeedR);
        }
        effectorLeft.set(0);
        effectorRight.set(0);
        effectorTimer.stop();
        effectorTimer.reset();
    }

    public static void manualControl(double velocityLeft, Double velocityRight) {
        if (velocityRight == null) {
            velocityRight = velocityLeft;
        }
        effectorLeft.set(velocityLeft);
        effectorRight.set(velocityRight);
    }

    public static void algaeEffectorUp() {
        RelativeEncoder algaeEncoder = algaeMotor.getEncoder();

        while (algaeEncoder.getPosition() < 0.0) {
            algaeMotor.set(100);
        }
        algaeMotor.set(0);
        isAlgaeOut = true;
    }

    public static void algaeEffectorDown() {
        RelativeEncoder algaeEncoder = algaeMotor.getEncoder();

        while (algaeEncoder.getPosition() > -0.025) {
            algaeMotor.set(-100);
        }
        algaeMotor.set(0);
        isAlgaeOut = false;
    }

    public static void toggleAlgae() {
        if (isAlgaeOut) {
            algaeEffectorDown();
        }
        else {
            algaeEffectorUp();
        }
    }
}
