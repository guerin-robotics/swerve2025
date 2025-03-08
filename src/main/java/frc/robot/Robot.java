package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.Command;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.LaserCan;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.*;

import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.subsystems.*;


public class Robot extends TimedRobot {
    private final static CANBus canbus = new CANBus("rio");
    private final static TalonFX m_liftLeft = new TalonFX(10, canbus);
    private final TalonFX m_liftRight = new TalonFX(11, canbus);
    private final TalonFX m_intakeLeft = new TalonFX(12, canbus);
    private final TalonFX m_intakeRight = new TalonFX(13, canbus);
    // private final DigitalInput toplimitSwitch = new DigitalInput(1);
    // private final static DigitalInput bottomlimitSwitch = new DigitalInput(0);
    // private final LaserCan elevatorBottom = new LaserCan(0);
    // private final LaserCan elevatorTop = new LaserCan(1);
    private final LaserCan intakeSensor = new LaserCan(2);
    private final SparkMax algaeMotor = new SparkMax(1, MotorType.kBrushed);
    

    // Thread Pool
    private ExecutorService executorService = Executors.newSingleThreadExecutor();
    /* Be able to switch which control request to use based on a button press */
    /* Start at velocity 0, use slot 0 */
    private final static VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final VelocityVoltage m_intakeVoltage = new VelocityVoltage(0).withSlot(1);
    /* Keep a neutral out so we can disable the motor */
    private final static NeutralOut m_brake = new NeutralOut();

    private final XboxController m_joystick = new XboxController(1);
    private final Joystick buttonPanel = new Joystick(2);

    private final Mechanisms m_mechanisms = new Mechanisms();
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private Elevator elevator;
    private DutyCycleOut control;
    private Timer timer;
    private Joystick joystick = new Joystick(0);

    private volatile static boolean isLiftMoving = false;
    private volatile boolean isIntaking = false;
    private volatile boolean isOuttaking = false;
    private volatile boolean isAlgaeOut = false;

    Timer intakeTimer = new Timer();
    static Timer liftTimer = new Timer();
    
    public Robot() {
        // Initialize robot components
        m_robotContainer = new RobotContainer();
        // LiftLeftTalonFX = new TalonFX(10, "rio");
        // LiftRightTalonFX = new TalonFX(11, "rio");
        control = new DutyCycleOut(0);
        timer = new Timer();
        timer.start();
        joystick = new Joystick(0);
        CameraServer.startAutomaticCapture();
        // elevator = new Elevator();

        CanBridge.runTCP();

    TalonFXConfiguration configs = new TalonFXConfiguration();
    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();

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
        // status = m_liftLeft.getConfigurator().apply(configs);
        m_intakeLeft.getConfigurator().apply(intakeConfig);
        if (status.isOK()) break;
    }
        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    
        // m_liftRight.setControl(new Follower(m_liftLeft.getDeviceID(), true));
        m_intakeRight.setControl(new Follower(m_intakeLeft.getDeviceID(), true));
        m_liftLeft.setPosition(0);
    
        }
    
    
    private void startIntakeSequence() {
        executorService.execute(() -> {
            try {
                intakeTimer.start();
                while (intakeSensor.getMeasurement().distance_mm > 10 && intakeTimer.get() < 5.00) {
                    m_intakeLeft.setControl(m_intakeVoltage.withVelocity(20));
                }
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                intakeTimer.stop();
                intakeTimer.reset();
                isIntaking = false;
            }
        });
    }
    
    private void startOuttakeSequence() {
        executorService.execute(() -> {
            try {
                intakeTimer.start();
                while (intakeTimer.get() < 1.00) {
                    m_intakeLeft.setControl(m_intakeVoltage.withVelocity(20));
                }
            } catch (Exception e) {
                e.printStackTrace();
            } finally {
                intakeTimer.stop();
                intakeTimer.reset();
                isOuttaking = false;
            }
        });
    }

    private void algaeIn() {
        // RelativeEncoder algaeEncoder = algaeMotor.getEncoder();
        RelativeEncoder algaeEncoder = algaeMotor.getEncoder();
        executorService.execute(() -> {
            try {
                while (algaeEncoder.getPosition() > -0.025) {
                    System.out.println(algaeEncoder.getPosition());
                    algaeMotor.set(100);
                }
                algaeMotor.set(0);
            }
            catch (Exception e) {
                e.printStackTrace();
            }
            finally {
                isAlgaeOut = false;
            }
        });
    }
    private void algaeOut() {
        RelativeEncoder algaeEncoder = algaeMotor.getEncoder();
        executorService.execute(() -> {
            try {
                while (algaeEncoder.getPosition() < 0.0) {
                    System.out.println(algaeEncoder.getPosition());
                    algaeMotor.set(-100);
                }
                algaeMotor.set(0);
            }
            catch (Exception e) {
                e.printStackTrace();
            }
            finally {
                isAlgaeOut = true;
            }
        });
    }
    @Override
    public void robotPeriodic() {
        // Runs the scheduler for commands
        m_mechanisms.update(m_liftLeft.getPosition(), m_liftLeft.getVelocity());
        CommandScheduler.getInstance().run();
        // if (elevatorBottom.getMeasurement() != null) {
        //     System.out.println("Elevator is " + elevatorBottom.getMeasurement().distance_mm + " mm away!");
        // }
    }


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
    if (executorService.isShutdown() || executorService.isTerminated()) {
        executorService = Executors.newSingleThreadExecutor();  // Restart executor
    }

    if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
    }
}

    @Override
public void teleopPeriodic() {

    double joyValue = m_joystick.getLeftY();
    double intakeSpeed = m_joystick.getRightY();
    if (Math.abs(joyValue) < 0.05) joyValue = 0; // add a deadband

    double intakeRotationsPerSecond = intakeSpeed * -50;

    if (m_joystick.getRightBumperButton()) {
        m_intakeLeft.setControl(m_intakeVoltage.withVelocity(intakeRotationsPerSecond * 2));
    } else {
        m_intakeLeft.setControl(m_intakeVoltage.withVelocity(0));
    }

    if (m_joystick.getRightTriggerAxis() > 0) {
        m_intakeLeft.setControl(m_intakeVoltage.withVelocity(20 * m_joystick.getRightTriggerAxis()));
    } else if (m_joystick.getLeftTriggerAxis() > 0) {
        m_intakeLeft.setControl(m_intakeVoltage.withVelocity(-20 * m_joystick.getLeftTriggerAxis()));
    }

    if (m_joystick.getAButton()) {
        m_intakeLeft.setControl(m_intakeVoltage.withVelocity(85));
        m_intakeRight.setControl(m_intakeVoltage.withVelocity(0));
    } else {
        m_intakeRight.setControl(new Follower(m_intakeLeft.getDeviceID(), true));
    }

    // System.out.println(Elevator.calculateSpeed(10, 50));

    // --- BUTTON PANEL ACTIONS (Now using flags) ---
    if (buttonPanel.getRawButtonPressed(1)) {
        if (!isIntaking) {
            isIntaking = true;
            startIntakeSequence();
        }
    }

    if (buttonPanel.getRawButtonPressed(7)) {
        if (!isOuttaking) {
            isOuttaking = true;
            startOuttakeSequence();
        }
    }

    if (buttonPanel.getRawButtonPressed(5)) {
        // moveLiftToPosition(35, 34.5);
        // .toPosition(35);

        if (!isAlgaeOut) {
            isAlgaeOut = true;
            algaeOut();
        }
        
    }

    if (buttonPanel.getRawButtonPressed(9)) {
        // moveLiftToPosition(50, 49.5);
        // Elevator.toPosition(50);
        if (!isAlgaeOut) {
            isAlgaeOut = true;
            algaeOut();
        }
    }

    if (buttonPanel.getRawButtonPressed(10)){
        if (isAlgaeOut) {
            isAlgaeOut = false;
            algaeIn();
        }
        else if (!isAlgaeOut) {
            isAlgaeOut = true;
            algaeOut();
        }
    }

}

    @Override
    public void disabledInit() {
        executorService.shutdownNow();  // Force stop all running tasks
    }
}
