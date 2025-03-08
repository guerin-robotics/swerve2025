package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.LaserCan;
import frc.robot.Constants;
import frc.robot.Constants.effector;
import frc.robot.Constants.elevator;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Effector extends SubsystemBase {
    private static TalonFX effectorLeft;
    private static TalonFX effectorRight;

    private static Timer intakeTimer = new Timer();

    private final static MotionMagicVelocityVoltage motionControl = new MotionMagicVelocityVoltage(0);

    public Effector() {
        effectorLeft = new TalonFX(Constants.effector.EffectorLeft);
        effectorRight = new TalonFX(Constants.effector.EffectorRight);
    }

    public void ConfigureEffector() {
        TalonFXConfiguration effectorConfig = new TalonFXConfiguration();
        var limitConfigs = new CurrentLimitsConfigs();

        effectorConfig.Slot1.kS = 0; // Static friction
        effectorConfig.Slot1.kV = 0.12; // 0.12 for Kraken X60
        effectorConfig.Slot1.kP = 0; // Rotational error per second
        effectorConfig.Slot1.kI = 0; // Integrated error
        effectorConfig.Slot1.kD = 0; // Error derivative

        effectorConfig.Voltage.withPeakForwardVoltage(Volts.of(8)).withPeakReverseVoltage(Volts.of(-8));

        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < 5; ++i) {
            status = effectorLeft.getConfigurator().apply(effectorConfig);
            if (status.isOK()) break;
        }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }

    public static void intakeSequence() {
        intakeTimer = new Timer();
        
    }

    public static void outtakeSequence() {

    }

    public static void scoreCoral(int level) {

    }

    public static void removeAlgae(int level) {

    }


}
