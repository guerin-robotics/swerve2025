package frc.robot;

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

public class Robot extends TimedRobot {
    private CommandSwerveDrivetrain drivetrain;
    private Command m_autonomousCommand;
    private RobotContainer m_robotContainer;
    private Vision vision;

    public Robot() {
        // Initialize robot components
        var inst = NetworkTableInstance.getDefault();
        inst.startServer();
        PathfindingCommand.warmupCommand().schedule();
        System.out.println("Starting NetworkTables server on port " + inst);
    }

    @Override
    public void robotPeriodic() {
        // Runs the scheduler for commands
        CommandScheduler.getInstance().run();
        vision.periodic();
        drivetrain.periodic();
        
        // Raw-vision telemetry: publish camera-only pose
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

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void robotInit() {
        m_robotContainer = new RobotContainer();
        drivetrain = m_robotContainer.drivetrain;
        vision = m_robotContainer.vision;
        CanBridge.runTCP();
        CameraServer.startAutomaticCapture();

    }

    public void resetPose() {
        // Example Only - startPose should be derived from some assumption
        // of where your robot was placed on the field.
        // The first pose in an autonomous path is often a good choice.

        Pose2d startPose = new Pose2d(7.89, 4.026, Rotation2d.fromDegrees(180)); // id 21
        // Pose2d startPose = new Pose2d(6, 5.5, Rotation2d.fromDegrees(240)); // id 20
        // Pose2d startPose = new Pose2d(4, 5.5, Rotation2d.fromDegrees(30)); // id 22

        drivetrain.resetPose(startPose);
        System.out.println("Resetting pose to " + startPose);

    }
}