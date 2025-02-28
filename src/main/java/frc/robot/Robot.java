package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.sim.PhysicsSim;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.ConfigurationFailedException;

public class Robot extends TimedRobot {
    private final CANBus canbus = new CANBus("rio");
    private final TalonFX m_fx = new TalonFX(10, canbus);
    private final TalonFX m_fllr = new TalonFX(11, canbus);
    private final TalonFX m_intakeLeft = new TalonFX(12, canbus);
    private final TalonFX m_intakeRight = new TalonFX(13, canbus);
    private final DigitalInput toplimitSwitch = new DigitalInput(0);
    private final DigitalInput bottomlimitSwitch = new DigitalInput(1);
    private LaserCan elevatorBottom = new LaserCan(0);
    private LaserCan elevatorTop = new LaserCan(1);
    private LaserCan intakeSensor = new LaserCan(2);

    /* Be able to switch which control request to use based on a button press */
    /* Start at velocity 0, use slot 0 */
    private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_intakeVoltage = new VelocityVoltage(0).withSlot(1);
    /* Keep a neutral out so we can disable the motor */
    private final NeutralOut m_brake = new NeutralOut();

    private final XboxController m_joystick = new XboxController(1);

    private final Mechanisms m_mechanisms = new Mechanisms();
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private TalonFX LiftLeftTalonFX;
    private TalonFX LiftRightTalonFX;
    private DutyCycleOut control;
    private Timer timer;
    private Joystick joystick;

    public Robot() {
        // Initialize robot components
        m_robotContainer = new RobotContainer();
        // LiftLeftTalonFX = new TalonFX(10, "rio");
        // LiftRightTalonFX = new TalonFX(11, "rio");
        control = new DutyCycleOut(0);
        timer = new Timer();
        timer.start();
        joystick = new Joystick(0);
        

        CanBridge.runTCP();

    TalonFXConfiguration configs = new TalonFXConfiguration();
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    configs.Slot0.kS = 0.3; // To account for friction, add 0.1 V of static feedforward
    configs.Slot0.kG = 0.2; // Gravity constant, determined by gear ratio
    configs.Slot0.kV = 0.13; // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    configs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 0.11 V output
    configs.Slot0.kI = 0.1; // No output for integrated error
    configs.Slot0.kD = 0.001; // No output for error derivative
    // Peak output of 8 volts
    configs.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    // configs.Slot1.kS = 2.5; // To account for friction, add 2.5 A of static feedforward
    // configs.Slot1.kP = 5; // An error of 1 rotation per second results in 5 A output
    // configs.Slot1.kI = 0; // No output for integrated error
    // configs.Slot1.kD = 0; // No output for error derivative
    // Peak output of 40 A
    configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
      .withPeakReverseTorqueCurrent(Amps.of(-40));

    intakeConfig.Slot1.kS = 0; // Static friction
    intakeConfig.Slot1.kV = 0.12; // 0.12 for Kraken X60
    intakeConfig.Slot1.kP = 0; // Rotational error per second
    intakeConfig.Slot1.kI = 0; // Integrated error
    intakeConfig.Slot1.kD = 0; // Error derivative

    intakeConfig.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));


    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_fx.getConfigurator().apply(configs);
      m_intakeLeft.getConfigurator().apply(intakeConfig);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

    m_fllr.setControl(new Follower(m_fx.getDeviceID(), true));
    m_intakeRight.setControl(new Follower(m_intakeLeft.getDeviceID(), true));
    m_fx.setPosition(0);
    }

    @Override
    public void robotPeriodic() {
        // Runs the scheduler for commands
        m_mechanisms.update(m_fx.getPosition(), m_fx.getVelocity());
        CommandScheduler.getInstance().run();
        // if (elevatorBottom.getMeasurement() != null) {
        //     System.out.println("Elevator is " + elevatorBottom.getMeasurement().distance_mm + " mm away!");
        // }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        // Start the selected autonomous command
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        // Stop autonomous command when teleop starts
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        double joyValue = m_joystick.getLeftY();
        double intakeSpeed = m_joystick.getRightY();
        if (Math.abs(joyValue) < 0.05) joyValue = 0; // add a deadband

        double desiredRotationsPerSecond;
        double intakeRotationsPerSecond = intakeSpeed * -50;

        double liftPosition = m_fx.getPosition().getValueAsDouble();
        System.out.println("Top: " + elevatorTop.getMeasurement().distance_mm);
        System.out.println("Bottom: " + elevatorBottom.getMeasurement().distance_mm);
        if ((elevatorTop.getMeasurement().distance_mm <= 40) && (joyValue < 0)) {
            desiredRotationsPerSecond = 0;
        }
        // else if ((elevatorTop.getMeasurement().distance_mm < 200) && (joyValue < 0)) {
        //     desiredRotationsPerSecond = joyValue * -(75-liftPosition)*10;
        // }
        // else if ((elevatorBottom.getMeasurement().distance_mm < 300) && (joyValue > 0)) { 
        //     desiredRotationsPerSecond = joyValue * -liftPosition*10;
        // }
        else if ((liftPosition > 60) && (joyValue < 0)) {
            desiredRotationsPerSecond = joyValue * -(75-liftPosition)*10;
        }
        else if ((liftPosition < 9) && (joyValue > 0)) { 
            desiredRotationsPerSecond = joyValue * -liftPosition*10;
        }
        else {
            desiredRotationsPerSecond = joyValue * -100;
        }

        if (!toplimitSwitch.get() && (joyValue > 0)) {
        
        m_fx.setControl(m_brake);
        } else if (!bottomlimitSwitch.get() && (joyValue < 0)) {
        m_fx.setControl(m_brake);
        } else {
        if (m_joystick.getLeftBumperButton()) {
            /* Use velocity voltage */
            m_fx.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
        } else {
            /* Disable the motor instead */
            m_fx.setControl(m_velocityVoltage.withVelocity(0));
        }
        }

        if (m_joystick.getRightBumperButton()) {
            /* Use velocity voltage */
            // if (intakeRotationsPerSecond > 0) {
            //     intakeRotationsPerSecond *= 0.25;
            // }
            m_intakeLeft.setControl(m_intakeVoltage.withVelocity(intakeRotationsPerSecond * 2));
        } else {
            /* Disable the motor instead */
            m_intakeLeft.setControl(m_intakeVoltage.withVelocity(0));
        }

        
        if (m_joystick.getRightTriggerAxis() > 0) {
            m_intakeLeft.setControl(m_intakeVoltage.withVelocity(20 * m_joystick.getRightTriggerAxis()));
        }
        else if (m_joystick.getLeftTriggerAxis() > 0) {
            m_intakeLeft.setControl(m_intakeVoltage.withVelocity(-20 * m_joystick.getLeftTriggerAxis()));
        }

        if (m_joystick.getAButton()) {
            m_intakeLeft.setControl(m_intakeVoltage.withVelocity(85));
            m_intakeRight.setControl(m_intakeVoltage.withVelocity(0));
        }
        else {
            m_intakeRight.setControl(new Follower(m_intakeLeft.getDeviceID(), true));
        }
    }

    @Override
    public void testInit() {
        // Cancel all running commands when test mode starts
        CommandScheduler.getInstance().cancelAll();
    }
}
