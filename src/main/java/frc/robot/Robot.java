package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Effector;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.controls.NeutralOut;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.LaserCan;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hang;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private XboxController m_joystick = new XboxController(1);

    private final LaserCan elevatorTop = new LaserCan(1);

    private final NeutralOut m_brake = new NeutralOut();

    Timer intakeTimer = new Timer();
    static Timer liftTimer = new Timer();
    
    public Robot() {
        // Initialize robot components
        m_robotContainer = new RobotContainer();
        CameraServer.startAutomaticCapture();
        // elevator = new Elevator();

        CanBridge.runTCP();
    }

    @Override
    public void robotPeriodic() {
        // Runs the scheduler for commands
        CommandScheduler.getInstance().run();
        if (RobotContainer.XboxController.getRawAxis(Constants.XboxController.axis.LeftTrigger) > 0) {
            Effector.manualControl(-0.5*70* RobotContainer.XboxController.getRawAxis(Constants.XboxController.axis.LeftTrigger), null);
        }
        else {
        Effector.manualControl(0.5*70*RobotContainer.XboxController.getRawAxis(Constants.XboxController.axis.RightTrigger), null);
        }
        if (RobotContainer.XboxController.button(Constants.XboxController.button.A).getAsBoolean()) {
            Effector.manualControl(20.0, -6.0);
        }
        
        SmartDashboard.putNumber("Tx reading", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(1));
        SmartDashboard.putNumber("Tv reading", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(1));
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
        // RobotContainer.drivetrain.seedFieldCentric(); // Not currently working - reverses direction on blue side
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        Constants.setL4();
        
    }

    @Override
    public void teleopPeriodic() {
        // double joyValue = m_joystick.getLeftY();
        // double intakeSpeed = m_joystick.getRightY();
        // if (Math.abs(joyValue) < 0.05) joyValue = 0; // add a deadband

        // double desiredRotationsPerSecond;
        // double intakeRotationsPerSecond = intakeSpeed * -50;
        // double liftPosition = Elevator.liftLeft.getPosition().getValueAsDouble();

        // if ((joyValue < 0)) {
        //     desiredRotationsPerSecond = 0;
        // } else if ((liftPosition > 60) && (joyValue < 0)) {
        //     desiredRotationsPerSecond = joyValue * -(75 - liftPosition) * 10;
        // } else if ((liftPosition < 9) && (joyValue > 0)) {
        //     desiredRotationsPerSecond = joyValue * -liftPosition * 10;
        // } else {
        //     desiredRotationsPerSecond = joyValue * -10;
        // }

        // if (!Elevator.bottomlimitSwitch.get() && (joyValue > 0)) {
        //     Elevator.liftLeft.setControl(m_brake);
        // } else if (!Elevator.toplimitSwitch.get() && (joyValue < 0)) {
        //     Elevator.liftLeft.setControl(m_brake);
        // } else {
        //     if (m_joystick.getLeftBumperButton()) {
        //         System.out.println("Moving elevator with speed " + desiredRotationsPerSecond);
        //         Elevator.liftLeft.setControl(Elevator.m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
        //     } else {
        //         Elevator.liftLeft.setControl(Elevator.m_velocityVoltage.withVelocity(0));
        //     }
        // }
    }

    @Override
    public void disabledInit() {
        Hang.brakeHang();
    }
}
