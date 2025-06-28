package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.Effector;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.commands.PathfindingCommand;
import au.grapplerobotics.CanBridge;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import edu.wpi.first.cameraserver.CameraServer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.controls.NeutralOut;
import com.pathplanner.lib.commands.PathfindingCommand;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.LaserCan;
import frc.robot.subsystems.Hang;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private Vision vision;
    private CommandSwerveDrivetrain drivetrain;
    private XboxController m_joystick = new XboxController(1);

    private final LaserCan elevatorTop = new LaserCan(1);

    private final NeutralOut m_brake = new NeutralOut();

    //private Spark intakeActuator = new Spark(7);

    Timer intakeTimer = new Timer();
    static Timer liftTimer = new Timer();

    public Robot() {
        // Initialize robot components
        var inst = NetworkTableInstance.getDefault();
        inst.startServer();
        m_robotContainer = new RobotContainer();
        CameraServer.startAutomaticCapture();
        PathfindingCommand.warmupCommand().schedule();
        // elevator = new Elevator();

        CanBridge.runTCP();
    }

    @Override
    public void robotPeriodic() {
        // Runs the scheduler for commands
        CommandScheduler.getInstance().run();
        vision.periodic();
        drivetrain.periodic();
        if (RobotContainer.XboxController.getRawAxis(Constants.XboxController.axis.LeftTrigger) > 0) {
            Effector.manualControl(
                    -0.5 * 70 * RobotContainer.XboxController.getRawAxis(Constants.XboxController.axis.LeftTrigger),
                    null);
        } else {
            Effector.manualControl(
                    0.5 * 70 * RobotContainer.XboxController.getRawAxis(Constants.XboxController.axis.RightTrigger),
                    null);
        }
        if (RobotContainer.XboxController.button(Constants.XboxController.button.A).getAsBoolean()) {
            Effector.manualControl(20.0, -6.0);
        }
    }
    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        drivetrain = m_robotContainer.drivetrain;
        vision = m_robotContainer.vision;
        CanBridge.runTCP();
        CameraServer.startAutomaticCapture();
    }

    

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void autonomousInit() {
        // Start the selected autonomous command
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
        // RobotContainer.drivetrain.seedFieldCentric(); // Not currently working -
        // reverses direction on blue side
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // resetPose();
        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
        Constants.setL4();

    }

    @Override
    public void teleopPeriodic() {

        // intakeActuator.set(-1); // Testing only, remove later. Only use 1 or -1.

    }

    @Override
    public void disabledInit() {
        Hang.brakeHang();
    }

    public void resetPose() {
        // Example Only - startPose should be derived from some assumption
        // of where your robot was placed on the field.
        // The first pose in an autonomous path is often a good choice.

        Pose2d startPose = new Pose2d(14.89, 4.026, Rotation2d.fromDegrees(180)); // id ??

        // Pose2d startPose = new Pose2d(7.89, 4.026, Rotation2d.fromDegrees(180)); // id 21
        // Pose2d startPose = new Pose2d(6, 5.5, Rotation2d.fromDegrees(240)); // id 20
        // Pose2d startPose = new Pose2d(4, 5.5, Rotation2d.fromDegrees(30)); // id 22

        drivetrain.resetPose(startPose);
        System.out.println("Resetting pose to " + startPose);

    }

}