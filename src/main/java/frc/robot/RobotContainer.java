// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.generated.TunerConstants;
import frc.robot.util.tagSide;
import frc.robot.util.TagUtils;

import static frc.robot.Constants.stearingMultiplier;

public class RobotContainer {

  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final SendableChooser<Command> autoChooser;

  public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 1; // kSpeedAt12Volts desired top
                                                                                          // speed
  public static double MaxAngularRate = RotationsPerSecond.of(2.5).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                         // max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);
  public final Vision vision;

  public static final CommandJoystick joystick = new CommandJoystick(0);
  public static final CommandJoystick XboxController = new CommandJoystick(1);
  public static final CommandJoystick buttonPanel = new CommandJoystick(2);

  final DutyCycleOut m_leftRequest = new DutyCycleOut(0.0);
  final DutyCycleOut m_rightRequest = new DutyCycleOut(0.0);

  public Elevator m_elevator = new Elevator();
  public Effector m_effector = new Effector();

  public RobotContainer() {
    drivetrain.seedFieldCentric();
    vision = new Vision((pose, timestamp, stdDevs) -> drivetrain.addVisionMeasurement(pose, timestamp, stdDevs));
    configureBindings();

    NamedCommands.registerCommand("scoreL1Coral",
        new SequentialCommandGroup(
            new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L1 + 2.0), m_elevator),
            new InstantCommand(() -> Effector.asymmetricalOuttake(null, null), m_effector),
            new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L1), m_elevator)));
    NamedCommands.registerCommand("scoreL4Coral",
        new SequentialCommandGroup(
            new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L4 - 1), m_elevator),
            new WaitCommand(1.5), new InstantCommand(() -> Effector.symmetricalOuttake(null), m_effector),
            new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L1), m_elevator)));
    NamedCommands.registerCommand("intakeCoral", new InstantCommand(() -> Effector.intakeUntilDetected()));

    autoChooser = AutoBuilder.buildAutoChooser("");
    SmartDashboard.putData("Auto Chooser", autoChooser);

  }

  private Command makeGoToTag(
      int tagId,
      tagSide side,
      double offsetMeters,
      double frontOffsetMeters) {
    // 1) compute the goal pose with your two offsets
    Pose2d goal = TagUtils.computeTagAdjacencyPose(
        tagId,
        side,
        offsetMeters,
        frontOffsetMeters);
    Logger.debug("computed adjacency goal: {}", goal);

    // 2) build your PathPlanner command as before
    PathConstraints constraints = new PathConstraints(
        Constants.Pathfinding.MaxSpeed,
        Constants.Pathfinding.MaxAccel,
        Units.degreesToRadians(Constants.Pathfinding.MaxRotSpeed),
        Units.degreesToRadians(Constants.Pathfinding.MaxRotAccel));
    Command pathCmd = AutoBuilder.pathfindToPose(goal, constraints, 0.0);

    // 3) make a "canceller" that finishes as soon as the stick moves >40%
    Command cancelOnStick = waitUntil(() -> Math.abs(joystick.getY()) > 0.2 ||
        Math.abs(joystick.getX()) > 0.2 ||
        Math.abs(joystick.getTwist()) > 0.2);

    // 4) race them: whichever completes first wins and cancels the other
    return race(pathCmd, cancelOnStick);
  }


  private Command goToLastSeenTag(tagSide side, double offsetMeters, double frontoffsetMeters) {
    return new InstantCommand(() -> {
      int tagId = vision.getLastSeenTagId();
      if (tagId < 0) {
        Logger.warn("No tag seen");
        return;
      }
      makeGoToTag(tagId, side, offsetMeters, frontoffsetMeters).schedule();
    }, drivetrain);
  }

  private void configureBindings() {
    buttonPanel.button(Constants.buttonPanel.lift.L1)
        .onTrue(new SequentialCommandGroup(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L1))));
    buttonPanel.button(Constants.buttonPanel.lift.L2)
        .onTrue(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L2)));
    buttonPanel.button(Constants.buttonPanel.lift.L3)
        .onTrue(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L3)));
    buttonPanel.button(Constants.buttonPanel.lift.L4)
        .onTrue(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L4)));
    buttonPanel.button(Constants.buttonPanel.algae.Lower).onTrue(new InstantCommand(() -> Sequences.removeL2Algae()));
    buttonPanel.button(Constants.buttonPanel.algae.Upper).onTrue(new InstantCommand(() -> Sequences.removeL3Algae()));
    buttonPanel.button(Constants.buttonPanel.algae.Retract)
        .onTrue(new ParallelDeadlineGroup(new InstantCommand(() -> Effector.toggleAlgae(), m_effector)));
    buttonPanel.button(Constants.buttonPanel.coral.In).onTrue(new ParallelRaceGroup(new WaitCommand(5.00),
        new InstantCommand(() -> Effector.intakeUntilDetected(), m_effector)));
    buttonPanel.button(Constants.buttonPanel.coral.Out)
        .onTrue(new InstantCommand(() -> Effector.outtakeUntilDetected()));
    XboxController.button(Constants.XboxController.button.A).onTrue(
        new SequentialCommandGroup(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L1 + 2)))); //
    XboxController.button(Constants.XboxController.bumper.Right).whileTrue(new RunCommand(
        () -> Effector.manualControl(XboxController.getRawAxis(Constants.XboxController.axis.RightYAxis) * 10, null)));
    XboxController.button(Constants.XboxController.button.X)
        .onTrue(new InstantCommand(() -> Sequences.removeL2Algae()));
    XboxController.button(Constants.XboxController.button.B)
        .onTrue(new InstantCommand(() -> Sequences.removeL3Algae()));
    XboxController.button(Constants.XboxController.button.Y).onTrue(new InstantCommand(() -> Elevator.toPosition(0)));
    XboxController.pov(Constants.XboxController.dpad.Up)
        .onTrue(new InstantCommand(() -> Effector.algaeEffectorUp(null), m_effector));
    XboxController.pov(Constants.XboxController.dpad.Down)
        .onTrue(new InstantCommand(() -> Effector.algaeEffectorDown(), m_effector));
    joystick.button(Constants.Joystick.Function1).onTrue(new InstantCommand(() -> Effector.algaeEffectorDown()));
    joystick.button(Constants.Joystick.Function2).onTrue(new InstantCommand(() -> Effector.algaeEffectorUp(null)));
    joystick.button(Constants.Joystick.servoControl).onTrue(new InstantCommand(() -> Hang.brakeHang()));
    // joystick.button(Constants.Joystick.strafeLeft).onTrue(new InstantCommand(()
    // -> Vision.strafe(true)));
    // joystick.button(Constants.Joystick.strafeRight).onTrue(new InstantCommand(()
    // -> Vision.strafe(false)));
    XboxController.pov(Constants.XboxController.dpad.Left)
        .onTrue(new InstantCommand(() -> Elevator.manualOffset(true)));
    XboxController.pov(Constants.XboxController.dpad.Right)
        .onTrue(new InstantCommand(() -> Elevator.manualOffset(false)));

    XboxController.button(Constants.XboxController.bumper.Right)
        .onTrue(new InstantCommand(() -> Hang.activateHang(false)));
    XboxController.button(Constants.XboxController.bumper.Left)
        .onTrue(new InstantCommand(() -> Hang.activateHang(true)));
    XboxController.button(Constants.XboxController.bumper.Right).onFalse(new InstantCommand(() -> Hang.stopHang()));
    XboxController.button(Constants.XboxController.bumper.Left).onFalse(new InstantCommand(() -> Hang.stopHang()));

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(joystick.getY() * MaxSpeed) // Drive forward with negative Y
                                                                                      // (forward)
            .withVelocityY(joystick.getX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-joystick.getTwist() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    joystick.trigger().whileTrue(drivetrain.applyRequest(() -> brake));

    // reset the field-centric heading on middle button press
    joystick.button(2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    drivetrain.registerTelemetry(logger::telemeterize);

    // Button commands
    joystick.button(Constants.Joystick.strafeLeft).onTrue(goToLastSeenTag(tagSide.LEFT, 0.164338, 0.355)); // 0.44
    joystick.button(Constants.Joystick.strafeRight).onTrue(goToLastSeenTag(tagSide.RIGHT, 0.164338, 0.355)); // 0.44

  }

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    return autoChooser.getSelected();
  }
}
