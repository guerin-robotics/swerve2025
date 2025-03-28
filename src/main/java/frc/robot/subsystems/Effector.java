 package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.AbsoluteEncoder;

import au.grapplerobotics.LaserCan;
import frc.robot.Constants;
import frc.robot.Constants.effector;
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
    private static Timer algaeTimer = new Timer();

    private final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);

    private final static MotionMagicVoltage motionMagicLeft = new MotionMagicVoltage(0).withSlot(1);
    private final static MotionMagicVoltage motionMagicRight = new MotionMagicVoltage(0).withSlot(1);

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
        TalonFXConfiguration effectorLeftConfig = new TalonFXConfiguration();
        TalonFXConfiguration effectorRightConfig = new TalonFXConfiguration();

        MotionMagicConfigs effectorLeftMotion = effectorLeftConfig.MotionMagic;
        MotionMagicConfigs effectorRightMotion = effectorRightConfig.MotionMagic;
        effectorLeftMotion.withMotionMagicCruiseVelocity(RotationsPerSecond.of(30)).withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100));
        effectorRightMotion.withMotionMagicCruiseVelocity(RotationsPerSecond.of(10)).withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(100));

        var limitConfigs = new CurrentLimitsConfigs();

        effectorTimer = new Timer();

        effectorLeftConfig.Slot0.kS = 0; // Static friction
        effectorLeftConfig.Slot0.kV = 0; // 0.12 for Kraken X60
        effectorLeftConfig.Slot0.kP = 0.3; // Rotational error per second
        effectorLeftConfig.Slot0.kI = 0; // Integrated error
        effectorLeftConfig.Slot0.kD = 0; // Error derivative

        effectorLeftConfig.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

        effectorRightConfig.Slot0.kS = 0; // Static friction
        effectorRightConfig.Slot0.kV = 0; // 0.12 for Kraken X60
        effectorRightConfig.Slot0.kP = 0.3; // Rotational error per second
        effectorRightConfig.Slot0.kI = 0; // Integrated error
        effectorRightConfig.Slot0.kD = 0; // Error derivative

        effectorRightConfig.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

        effectorLeftConfig.Slot1.kS = 1; // Static friction
        effectorLeftConfig.Slot1.kV = 0; // 0.12 for Kraken X60
        effectorLeftConfig.Slot1.kP = 1.0; // Rotational error per second
        effectorLeftConfig.Slot1.kI = 0; // Integrated error
        effectorLeftConfig.Slot1.kD = 0; // Error derivative

        effectorLeftConfig.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

        effectorRightConfig.Slot1.kS = 1; // Static friction
        effectorRightConfig.Slot1.kV = 0; // 0.12 for Kraken X60
        effectorRightConfig.Slot1.kP = 1.0; // Rotational error per second
        effectorRightConfig.Slot1.kI = 0; // Integrated error
        effectorRightConfig.Slot1.kD = 0; // Error derivative

        effectorRightConfig.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            // status = effectorLeft.getConfigurator().apply(effectorConfig);
            // effectorRight.getConfigurator().apply(effectorConfig);
            status = effectorRight.getConfigurator().apply(effectorRightConfig);
            effectorLeft.getConfigurator().apply(effectorLeftConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    public static void intakeUntilDetected() {
        effectorTimer.start();
        while (intakeSensor.getMeasurement().distance_mm > 10) {
            if (effectorTimer.get() > 5) {
                break;
            }
            else {
            effectorLeft.setControl(m_velocityVoltage.withVelocity(25));
            effectorRight.setControl(m_velocityVoltage.withVelocity(-25));
            }
        }
        effectorTimer.stop();
        effectorTimer.reset();
        effectorTimer.start();
        while (effectorTimer.get() < 0.2) {
            effectorLeft.setControl(m_velocityVoltage.withVelocity(20));
            effectorRight.setControl(m_velocityVoltage.withVelocity(-20));
        }
        effectorLeft.setControl(m_velocityVoltage.withVelocity(0));
        effectorRight.setControl((m_velocityVoltage.withVelocity(0)));
        effectorTimer.stop();
        effectorTimer.reset();
    }

    public static void outtakeUntilDetected() {
        if (Elevator.getPosition() < 5) {
            asymmetricalOuttake(null, null);
        }
        else {
            while (intakeSensor.getMeasurement().distance_mm < 10) {
            symmetricalOuttake(null);
            }
        }
        effectorLeft.setControl(m_velocityVoltage.withVelocity(0));
        effectorRight.setControl(m_velocityVoltage.withVelocity(0));
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
            effectorLeft.setControl(m_velocityVoltage.withVelocity(motorSpeed));
            effectorRight.setControl(m_velocityVoltage.withVelocity(-motorSpeed));
        }
        effectorLeft.setControl(m_velocityVoltage.withVelocity(0));
        effectorRight.setControl(m_velocityVoltage.withVelocity(0));
        effectorTimer.stop();
        effectorTimer.reset();
        return;
    }

    public static void asymmetricalOuttake(Double velocityLeft, Double velocityRight) {
        System.out.println(effectorLeft.getPosition().getValueAsDouble());
        double motorSpeedL;
        double motorSpeedR;
        if (velocityLeft != null) {
            motorSpeedL = velocityLeft;
        }
        else {
            motorSpeedL = 30;
        }
        if (velocityRight != null) {
            motorSpeedR = velocityRight;
        }
        else {
            motorSpeedR = 10;
        }
        effectorTimer.start();
        while (effectorTimer.get() < 1.5) {
            effectorLeft.setControl(m_velocityVoltage.withVelocity(motorSpeedL));
            effectorRight.setControl(m_velocityVoltage.withVelocity(-motorSpeedR));
        }

        // effectorLeft.setControl(motionMagicLeft.withPosition(100));
        // effectorRight.setControl(motionMagicRight.withPosition(30));
        effectorLeft.setControl(m_velocityVoltage.withVelocity(0));
        effectorRight.setControl(m_velocityVoltage.withVelocity(0));
        effectorTimer.stop();
        effectorTimer.reset();
    }

    public static void manualControl(double velocityLeft, Double velocityRight) {
        if (velocityRight == null) {
            velocityRight = -velocityLeft;
        }
        effectorLeft.setControl(m_velocityVoltage.withVelocity(velocityLeft));
        // effectorRight.setControl(m_velocityVoltage.withVelocity(velocityRight));
    }

    public static void algaeEffectorUp(Double time) {
        algaeTimer.start();

        if (time == null) {
            time = 0.1; //0.10
        }
        else {
        }

        while (algaeTimer.get() < time) {
            algaeMotor.set(-80); //-80
        }
        algaeMotor.set(0);
        isAlgaeOut = true;

        algaeTimer.stop();
        algaeTimer.reset();

        // RelativeEncoder algaeEncoder = algaeMotor.getEncoder();

        // while (algaeEncoder.getPosition() < 0.028) {
        //     algaeMotor.set(-80);
        // }
        // algaeMotor.set(0);
        // isAlgaeOut = true;
    }

    public static void algaeEffectorDown() {
        algaeTimer.start();

        while (algaeTimer.get() < 0.08) { //0.08
            algaeMotor.set(80); //80
        }
        algaeMotor.set(0);
        isAlgaeOut = false;

        algaeTimer.stop();
        algaeTimer.reset();        
        
        // RelativeEncoder algaeEncoder = algaeMotor.getEncoder();

        // while (algaeEncoder.getPosition() > 0.1 // 0.1
        // ) {
        //     algaeMotor.set(80);
        // }
        // algaeMotor.set(0);
        // isAlgaeOut = false;
    }

    public static void resetAlgaePosition() {
        RelativeEncoder algaeEncoder = algaeMotor.getEncoder();

        algaeEncoder.setPosition(0);
    }

    public static void toggleAlgae() {
        if (isAlgaeOut) {
            algaeEffectorDown();
        }
        else {
            algaeEffectorUp(null);
        }
    }
}
