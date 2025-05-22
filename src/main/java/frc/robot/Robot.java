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

import edu.wpi.first.math.geometry.Pose2d;
import java.util.Optional;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import java.util.logging.LogRecord;

import com.ctre.phoenix6.controls.NeutralOut;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.ctre.phoenix6.SignalLogger;

import frc.robot.Telemetry;

import au.grapplerobotics.CanBridge;
import au.grapplerobotics.LaserCan;

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
        resetPose();

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
        Pose2d startPose = new Pose2d(7.89, 4.026, Rotation2d.fromDegrees(-180));
        drivetrain.resetPose(startPose);
        System.out.println("Resetting pose to " + startPose);

    }
}