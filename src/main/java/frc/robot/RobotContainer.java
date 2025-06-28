// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.ArrayList;
import java.util.Comparator;

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

import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.wpilibj2.command.Commands.*;

import frc.robot.util.tagSide;
import frc.robot.util.TagUtils;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.Timer;

public class RobotContainer {
    // Tracks the currently targeted AprilTag ID for stick-based rotation commands
    private int mCurrentTargetTag = -1;
    // Timestamp of the last strafe button press
    private double mLastStrafeButtonTime = 0;
    // Tracks which side was chosen on the last strafe button press
    private tagSide mCurrentTargetSide = tagSide.LEFT;

    // 1) Separate tag IDs into upper and lower groups
    private static final List<Integer> kUpperTags = List.of(6, 7, 8, 9, 10, 11);
    private static final List<Integer> kLowerTags = List.of(17, 18, 19, 20, 21, 22);

    // 2) Find the closest tag ID to the robot (search both groups)
    private int getClosestTagId() {
        Pose2d robotPose = drivetrain.getPose();
        List<Integer> allTags = new ArrayList<>(kUpperTags);
        allTags.addAll(kLowerTags);
        return allTags.stream()
                .min(Comparator.comparingDouble(id -> TagUtils.getTagPose2d(id)
                        .map(p -> p.getTranslation().getDistance(robotPose.getTranslation()))
                        .orElse(Double.MAX_VALUE)))
                .orElse(kUpperTags.get(0));
    }

    // 3) Get next tag in clockwise order within the same group
    private int getNextClockwise(int currentTagId) {
        if (kUpperTags.contains(currentTagId)) {
            int idx = kUpperTags.indexOf(currentTagId);
            return kUpperTags.get((idx + 1) % kUpperTags.size());
        } else if (kLowerTags.contains(currentTagId)) {
            int idx = kLowerTags.indexOf(currentTagId);
            return kLowerTags.get((idx + 1) % kLowerTags.size());
        } else {
            return getClosestTagId();
        }
    }

    // 4) Get previous tag in clockwise order within the same group
    private int getPrevClockwise(int currentTagId) {
        if (kUpperTags.contains(currentTagId)) {
            int idx = kUpperTags.indexOf(currentTagId);
            return kUpperTags.get((idx + kUpperTags.size() - 1) % kUpperTags.size());
        } else if (kLowerTags.contains(currentTagId)) {
            int idx = kLowerTags.indexOf(currentTagId);
            return kLowerTags.get((idx + kLowerTags.size() - 1) % kLowerTags.size());
        } else {
            return getClosestTagId();
        }
    }

    public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> autoChooser;

    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 1; // kSpeedAt12Volts desired
                                                                                            // top
                                                                                            // speed
    public static double MaxAngularRate = RotationsPerSecond.of(2.5).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                           // second
                                                                                           // max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

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
        Command cancelOnStick = waitUntil(() -> Math.abs(joystick.getY()) > 0.4 ||
                Math.abs(joystick.getX()) > 0.2 ||
                Math.abs(joystick.getTwist()) > 0.2);

        // 4) race them: whichever completes first wins and cancels the other
        return race(pathCmd, cancelOnStick);
    }

    private void configureBindings() {
        buttonPanel.button(Constants.buttonPanel.lift.L1)
                .onTrue(new SequentialCommandGroup(
                        new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L1))));
        buttonPanel.button(Constants.buttonPanel.lift.L2)
                .onTrue(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L2)));
        buttonPanel.button(Constants.buttonPanel.lift.L3)
                .onTrue(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L3)));
        buttonPanel.button(Constants.buttonPanel.lift.L4)
                .onTrue(new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L4)));
        buttonPanel.button(Constants.buttonPanel.algae.Lower)
                .onTrue(new InstantCommand(() -> Sequences.removeL2Algae()));
        buttonPanel.button(Constants.buttonPanel.algae.Upper)
                .onTrue(new InstantCommand(() -> Sequences.removeL3Algae()));
        buttonPanel.button(Constants.buttonPanel.algae.Retract)
                .onTrue(new ParallelDeadlineGroup(new InstantCommand(() -> Effector.toggleAlgae(), m_effector)));
        buttonPanel.button(Constants.buttonPanel.coral.In).onTrue(new ParallelRaceGroup(new WaitCommand(5.00),
                new InstantCommand(() -> Effector.intakeUntilDetected(), m_effector)));
        buttonPanel.button(Constants.buttonPanel.coral.Out)
                .onTrue(new InstantCommand(() -> Effector.outtakeUntilDetected()));
        buttonPanel.button(Constants.buttonPanel.intake.intakeDropButton).onTrue(new InstantCommand(() -> Hang.intakeDrop()));
        XboxController.button(Constants.XboxController.button.A).onTrue(
                new SequentialCommandGroup(
                        new InstantCommand(() -> Elevator.toPosition(Constants.elevator.level.L1 + 2)))); //
        XboxController.button(Constants.XboxController.bumper.Right).whileTrue(new RunCommand(
                () -> Effector.manualControl(XboxController.getRawAxis(Constants.XboxController.axis.RightYAxis) * 10,
                        null)));
        XboxController.button(Constants.XboxController.button.X)
                .onTrue(new InstantCommand(() -> Sequences.removeL2Algae()));
        XboxController.button(Constants.XboxController.button.B)
                .onTrue(new InstantCommand(() -> Sequences.removeL3Algae()));
        XboxController.button(Constants.XboxController.button.Y)
                .onTrue(new InstantCommand(() -> Elevator.toPosition(0)));
        XboxController.pov(Constants.XboxController.dpad.Up)
                .onTrue(new InstantCommand(() -> Effector.algaeEffectorUp(null), m_effector));
        XboxController.pov(Constants.XboxController.dpad.Down)
                .onTrue(new InstantCommand(() -> Effector.algaeEffectorDown(), m_effector));
        joystick.button(Constants.Joystick.Function1).onTrue(new InstantCommand(() -> Effector.algaeEffectorDown()));
        //joystick.button(Constants.Joystick.Function2).onTrue(new InstantCommand(() -> Effector.algaeEffectorUp(null)));
        joystick.button(Constants.Joystick.servoControl).onTrue(new InstantCommand(() -> Hang.brakeHang()));
        joystick.button(Constants.Joystick.intakeResetButton).onTrue(new InstantCommand(() -> Hang.intakeReset()));

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

        // choose your expo blend (0=no expo, 1=full cubic)

        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> {
                    double rawY = joystick.getY();
                    double rawX = joystick.getX();
                    double rawRot = -joystick.getTwist();

                    // cubic expo
                    double yCub = Math.copySign(rawY * rawY * rawY, rawY);
                    double xCub = Math.copySign(rawX * rawX * rawX, rawX);
                    double rCub = Math.copySign(rawRot * rawRot * rawRot, rawRot);

                    // blend linear + cubic
                    double yOut = Constants.stearingMultiplier * yCub + (1 - Constants.stearingMultiplier) * rawY;
                    double xOut = Constants.stearingMultiplier * xCub + (1 - Constants.stearingMultiplier) * rawX;
                    double rotOut = Constants.stearingMultiplier * rCub + (1 - Constants.stearingMultiplier) * rawRot;

                    return drive
                            .withVelocityX(yOut * MaxSpeed * Constants.masterDriveMultiplier)
                            .withVelocityY(xOut * MaxSpeed * Constants.masterDriveMultiplier)
                            .withRotationalRate(rotOut * MaxAngularRate * Constants.masterDriveMultiplier);
                }));

        joystick.trigger().whileTrue(drivetrain.applyRequest(() -> brake));

        // reset the field-centric heading on middle button press
        joystick.button(2).onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        drivetrain.registerTelemetry(logger::telemeterize);

        // Button commands
        joystick.button(Constants.Joystick.strafeLeft)
                .onTrue(new InstantCommand(() -> {
                    mLastStrafeButtonTime = Timer.getFPGATimestamp();
                    int closest = getClosestTagId();
                    mCurrentTargetTag = closest;
                    mCurrentTargetSide = tagSide.LEFT;
                    makeGoToTag(closest, tagSide.LEFT, 0.164338, 0.35).schedule();
                }, drivetrain));

        // Button commands and stick-based triggers for strafeRight and tag rotation
        if (!Constants.masterNerf) {
            joystick.button(Constants.Joystick.strafeRight)
                    .onTrue(new InstantCommand(() -> {
                        mLastStrafeButtonTime = Timer.getFPGATimestamp();
                        int closest = getClosestTagId();
                        mCurrentTargetTag = closest;
                        mCurrentTargetSide = tagSide.RIGHT;
                        makeGoToTag(closest, tagSide.RIGHT, 0.164338, 0.35).schedule();
                    }, drivetrain));

            // Stick-based clockwise rotation (>25%) within 5 seconds of last strafe button
            new Trigger(() -> joystick.getX() > 0.25
                    && Timer.getFPGATimestamp() - mLastStrafeButtonTime < 5.0)
                    .onTrue(new InstantCommand(() -> {
                        if (mCurrentTargetTag < 0) {
                            mCurrentTargetTag = getClosestTagId();
                        }
                        int next = getNextClockwise(mCurrentTargetTag);
                        mCurrentTargetTag = next;
                        makeGoToTag(next, mCurrentTargetSide, 0.164338, 0.35).schedule();
                    }, drivetrain));

            // Stick-based counter-clockwise rotation (<-25%) within 5 seconds of last
            // strafe button
            new Trigger(() -> joystick.getX() < -0.25
                    && Timer.getFPGATimestamp() - mLastStrafeButtonTime < 5.0)
                    .onTrue(new InstantCommand(() -> {
                        if (mCurrentTargetTag < 0) {
                            mCurrentTargetTag = getClosestTagId();
                        }
                        int prev = getPrevClockwise(mCurrentTargetTag);
                        mCurrentTargetTag = prev;
                        tagSide newSide = (mCurrentTargetSide == tagSide.LEFT) ? tagSide.RIGHT : tagSide.LEFT;
                        makeGoToTag(prev, newSide, 0.164338, 0.35).schedule();
                    }, drivetrain));
        }
    }

    public Command getAutonomousCommand() {
        // return Commands.print("No autonomous command configured");
        return autoChooser.getSelected();
    }
}
