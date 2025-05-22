package frc.robot;

import static frc.robot.generated.TunerConstants.createDrivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import static frc.robot.Constants.Vision.kOdometryUpdateHz;
import static frc.robot.Constants.Vision.kSingleTagStdDevs;
import static frc.robot.Constants.Vision.kMultiTagStdDevs;
import static frc.robot.Constants.Vision.kOdometryStdDevs;

import static frc.robot.Constants.Pathfinding.MaxSpeed;
import static frc.robot.Constants.Pathfinding.MaxAccel;
import static frc.robot.Constants.Pathfinding.MaxRotSpeed;
import static frc.robot.Constants.Pathfinding.MaxRotAccel;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;

public class RobotContainer {

    private final SendableChooser<Command> autoChooser;

    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 1; // kSpeedAt12Volts desired
                                                                                            // top speed
    public static double MaxAngularRate = RotationsPerSecond.of(2.5).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                           // second max angular
                                                                                           // velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    public final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandJoystick joystick = new CommandJoystick(0);

    Orchestra orchestra = new Orchestra();

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Vision vision;

    public RobotContainer() {
        drivetrain.seedFieldCentric();
        vision = new Vision((pose, timestamp, stdDevs) -> drivetrain.addVisionMeasurement(pose, timestamp, stdDevs));
        configureBindings();
        // Temporary “no-op” chooser until PathPlannerAutoBuilder is configured
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("No Auto", Commands.none());
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private Command makeGoToPose(double X, double Y, double rotation) {
        Pose2d targetPose = new Pose2d(X, Y, Rotation2d.fromDegrees(rotation));
        Logger.debug("pose [{}]", targetPose);
        PathConstraints constraints = new PathConstraints(
                frc.robot.Constants.Pathfinding.MaxSpeed, // max translation m/s
                frc.robot.Constants.Pathfinding.MaxAccel, // max accel m/s²
                Units.degreesToRadians(frc.robot.Constants.Pathfinding.MaxRotSpeed), // max rot rad/s
                Units.degreesToRadians(frc.robot.Constants.Pathfinding.MaxRotAccel) // max rot accel rad/s²
        );

        return AutoBuilder.pathfindToPose(
                targetPose,
                constraints,
                0.0);
    }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.

        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getY() * MaxSpeed) // Drive forward with
                                                                                              // negative Y (forward)
                        .withVelocityY(joystick.getX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-joystick.getTwist() * MaxAngularRate) // Drive counterclockwise with
                                                                                   // negative X (left)
                ));
        drivetrain.configureAutoBuilder();
        // Button commands

        joystick.button(Constants.Joystick.Function1).onTrue(makeGoToPose(7.89, 4.026, 180));

        joystick.trigger().whileTrue(drivetrain.applyRequest(() -> brake));

        // reset the field-centric heading on middle button press
        joystick.button(2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        return autoChooser.getSelected();
    }

}
