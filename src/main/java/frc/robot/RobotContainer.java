// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;


public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 1; // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public static final CommandJoystick joystick = new CommandJoystick(0);
    public static final CommandJoystick XboxController = new CommandJoystick(1);
    public static final CommandJoystick buttonPanel = new CommandJoystick(2);

    final DutyCycleOut m_leftRequest = new DutyCycleOut(0.0);
    final DutyCycleOut m_rightRequest = new DutyCycleOut(0.0);

    public Elevator m_elevator = new Elevator();
    public Effector m_effector = new Effector();


    Orchestra m_Orchestra = new Orchestra();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        drivetrain.seedFieldCentric();
        if (SwerveRequest.ForwardPerspectiveValue.OperatorPerspective == SwerveRequest.ForwardPerspectiveValue.BlueAlliance) {
            var heading = 1;
        }
        else {
            var heading = -1;
        }

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }

    private void configureBindings() {
        buttonPanel.button(Constants.buttonPanel.lift.L1).onTrue(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L1)));
        buttonPanel.button(Constants.buttonPanel.lift.L2).onTrue(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L2)));
        buttonPanel.button(Constants.buttonPanel.lift.L3).onTrue(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L3)));
        buttonPanel.button(Constants.buttonPanel.lift.L4).onTrue(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L4)));
        buttonPanel.button(Constants.buttonPanel.algae.Lower).onTrue(new InstantCommand(() -> Sequences.removeL2Algae()));
        buttonPanel.button(Constants.buttonPanel.algae.Upper).onTrue(new InstantCommand(() -> Sequences.removeL3Algae()));
        buttonPanel.button(Constants.buttonPanel.algae.Retract).onTrue(new InstantCommand(() -> Effector.toggleAlgae()));
        buttonPanel.button(Constants.buttonPanel.coral.In).onTrue(new InstantCommand(() -> Effector.intakeUntilDetected()));
        buttonPanel.button(Constants.buttonPanel.coral.Out).onTrue(new InstantCommand(() -> Effector.outtakeUntilDetected()));
        XboxController.button(Constants.XboxController.bumper.Left).whileTrue(new RunCommand(() -> Elevator.manualControl(XboxController.getRawAxis(Constants.XboxController.axis.LeftYAxis))));
        XboxController.button(Constants.XboxController.button.A).onTrue(new InstantCommand(() -> Effector.asymmetricalOuttake(null, null)));
        XboxController.button(Constants.XboxController.bumper.Right).whileTrue(new RunCommand(() -> Effector.manualControl(XboxController.getRawAxis(Constants.XboxController.axis.RightYAxis), null)));
        XboxController.button(Constants.XboxController.button.X).onTrue(new InstantCommand(() -> Sequences.removeL2Algae()));
        XboxController.button(Constants.XboxController.button.B).onTrue(new InstantCommand(() -> Sequences.removeL3Algae()));
        XboxController.button(Constants.XboxController.button.Y).onTrue(new InstantCommand(() -> Elevator.toPosition(0)));
        


        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.


        double xSpeed;
        double rot;
        // double ySpeed;
        var ySpeed = joystick.getY() * MaxSpeed;
        // if (Xbox.getBButtonPressed()) {
        //     System.out.println("Limelight activated");
        //     final var rot_limelight = limelight_aim_proportional();
        //     rot = rot_limelight;
        //     final var forward_limelight = limelight_range_proportional();
        //     xSpeed = forward_limelight; 
        // }
        // else {
        //     xSpeed = -joystick.getX() * MaxSpeed;

        //     rot = -joystick.getTwist() * MaxAngularRate;
        // }

        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getTwist() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.trigger().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.button(4).whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getY(), -joystick.getX()))
        ));


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on middle button press
        joystick.button(2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    // double limelight_aim_proportional() {
    //     double kP = 0.035;
    //     double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
    //     targetingAngularVelocity *= MaxSpeed;
    //     targetingAngularVelocity *= -1.0;
    //     return targetingAngularVelocity;
    // }

    // double limelight_range_proportional() {
    //     double kP = 0.1;
    //     double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    //     targetingForwardSpeed *= MaxSpeed;
    //     targetingForwardSpeed *= -1.0;
    //     return targetingForwardSpeed;
    // }

    private void align(boolean fieldRelative) {
        // var xSpeed = -joystick.getY() * MaxSpeed;
        var ySpeed = -joystick.getX() * MaxSpeed;
        // var rot = -joystick.getTwist() * MaxAngularRate;  
    }


    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        return autoChooser.getSelected();
    }
}

