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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;

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
        resetPose();
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
        int pos = m_robotContainer.positionChooser.getSelected();
        if (pos == 2) {
            System.out.println("Position 2 selected; skipping pose reset.");
            return;
        }
        Pose2d startPose;
        if (pos == 1) {
            startPose = new Pose2d(16, 4, Rotation2d.fromDegrees(0));
        } else if (pos == 3) {
            startPose = new Pose2d(14, 6, Rotation2d.fromDegrees(60)); // Add your third pose here
        } else {
            startPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
        }

        drivetrain.resetPose(startPose);
        System.out.println("Resetting pose to " + startPose);
    }

}