// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;


public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 1; // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
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

    public Vision m_vision = new Vision();


    Orchestra m_Orchestra = new Orchestra();

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public RobotContainer() {
        drivetrain.seedFieldCentric();
        if (SwerveRequest.ForwardPerspectiveValue.OperatorPerspective == SwerveRequest.ForwardPerspectiveValue.BlueAlliance) {
            var heading = 1;
        }
        else {
            var heading = -1;
        }

        configureBindings();

        NamedCommands.registerCommand("scoreL1Coral", new SequentialCommandGroup(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L1 + 2.0), m_elevator), new InstantCommand(() -> Effector.asymmetricalOuttake(null, null), m_effector), new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L1), m_elevator)));
        NamedCommands.registerCommand("scoreL4Coral", new SequentialCommandGroup(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L4 - 1), m_elevator), new WaitCommand(1.5), new InstantCommand(() -> Effector.symmetricalOuttake(null), m_effector), new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L1), m_elevator)));
        NamedCommands.registerCommand("intakeCoral", new InstantCommand(() -> Effector.intakeUntilDetected()));

        autoChooser = AutoBuilder.buildAutoChooser("");
        SmartDashboard.putData("Auto Chooser", autoChooser);
        
        
        // NamedCommands.registerCommand("Score L1", Effector.asymmetricalOuttake(null, null));
    }

    private void configureBindings() {
        buttonPanel.button(Constants.buttonPanel.lift.L1).onTrue(new SequentialCommandGroup(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L1))));
        buttonPanel.button(Constants.buttonPanel.lift.L2).onTrue(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L2)));
        buttonPanel.button(Constants.buttonPanel.lift.L3).onTrue(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L3)));
        buttonPanel.button(Constants.buttonPanel.lift.L4).onTrue(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L4)));
        buttonPanel.button(Constants.buttonPanel.algae.Lower).onTrue(new InstantCommand(() -> Sequences.removeL2Algae()));
        buttonPanel.button(Constants.buttonPanel.algae.Upper).onTrue(new InstantCommand(() -> Sequences.removeL3Algae()));
        //buttonPanel.button(Constants.buttonPanel.algae.Retract).onTrue(new InstantCommand(() -> Effector.toggleAlgae(), m_effector));
        buttonPanel.button(Constants.buttonPanel.algae.Retract).onTrue(new ParallelDeadlineGroup(new InstantCommand(() -> Effector.toggleAlgae(), m_effector)));
        buttonPanel.button(Constants.buttonPanel.coral.In).onTrue(new ParallelRaceGroup(new WaitCommand(5.00), new InstantCommand(() -> Effector.intakeUntilDetected(), m_effector)));
        buttonPanel.button(Constants.buttonPanel.coral.Out).onTrue(new InstantCommand(() -> Effector.outtakeUntilDetected()));
        // XboxController.button(Constants.XboxController.bumper.Left).whileTrue(new InstantCommand(() -> Elevator.manualControl(XboxController.getRawAxis(Constants.XboxController.axis.LeftYAxis)*10)));
        XboxController.button(Constants.XboxController.button.A).onTrue(new SequentialCommandGroup(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L1 + 2)), new InstantCommand(() -> Effector.asymmetricalOuttake(null, null)), new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L1))));
        XboxController.button(Constants.XboxController.bumper.Right).whileTrue(new RunCommand(() -> Effector.manualControl(XboxController.getRawAxis(Constants.XboxController.axis.RightYAxis)*10, null)));
        XboxController.button(Constants.XboxController.button.X).onTrue(new InstantCommand(() -> Sequences.removeL2Algae()));
        XboxController.button(Constants.XboxController.button.B).onTrue(new InstantCommand(() -> Sequences.removeL3Algae()));
        XboxController.button(Constants.XboxController.button.Y).onTrue(new InstantCommand(() -> Elevator.toPosition(0)));
        XboxController.button(Constants.XboxController.button.Start).whileTrue(new RunCommand(() -> Vision.applyLimelight(MaxAngularRate)));
        XboxController.pov(Constants.XboxController.dpad.Up).onTrue(new InstantCommand(() -> Effector.algaeEffectorUp(null), m_effector));
        XboxController.pov(Constants.XboxController.dpad.Down).onTrue(new InstantCommand(() -> Effector.algaeEffectorDown(), m_effector));
        joystick.button(Constants.Joystick.Function1).onTrue(new InstantCommand(() -> Effector.algaeEffectorDown()));
        joystick.button(Constants.Joystick.Function2).onTrue(new InstantCommand(() -> Effector.algaeEffectorUp(null)));
        XboxController.pov(Constants.XboxController.dpad.Left).onTrue(new InstantCommand(() -> Elevator.manualOffset(false)));
        XboxController.pov(Constants.XboxController.dpad.Right).onTrue(new InstantCommand(() -> Elevator.manualOffset(true)));


        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.


        double xSpeed;
        double rot;
        // double ySpeed;
        var ySpeed = joystick.getY() * MaxSpeed;
        
        // if (XboxController.button(Constants.XboxController.button.Start)) {
        //     System.out.println("Limelight activated");
        //     final var rot_limelight = m_vision.limelight_aim_proportional(MaxAngularRate);
        //     rot = rot_limelight;
        //     final var forward_limelight = m_vision.limelight_range_proportional(MaxAngularRate);
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

