package frc.robot;

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
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.util.TagSide;
import frc.robot.util.TagUtils;

import static frc.robot.Constants.stearingMultiplier;

public class RobotContainer {
  public final static CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 1; // kSpeedAt12Volts desired
                                                                                          // top speed
  public static double MaxAngularRate = RotationsPerSecond.of(1.75).in(RadiansPerSecond); // 3/4 of a rotation per
                                                                                          // second max angular
                                                                                          // velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  public static final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 1% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

  public static final CommandJoystick joystick = new CommandJoystick(0);

  private final SendableChooser<Command> autoChooser;
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  public final Telemetry logger = new Telemetry(MaxSpeed);

  Orchestra orchestra = new Orchestra(); // TODO is this used? Move to constructor

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

  private Command makeGoToTag(
      int tagId,
      TagSide side,
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

  private Command goToLastSeenTag(TagSide side, double offsetMeters, double frontoffsetMeters) {
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

    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.

    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> {
          double rawY = -joystick.getY();
          double rawX = -joystick.getX();
          double rawRot = -joystick.getTwist();

          // cubic expo
          double yCub = Math.copySign(rawY * rawY * rawY, rawY);
          double xCub = Math.copySign(rawX * rawX * rawX, rawX);
          double rCub = Math.copySign(rawRot * rawRot * rawRot, rawRot);

          // blend linear + cubic
          double yOut = stearingMultiplier * yCub + (1 - stearingMultiplier) * rawY;
          double xOut = stearingMultiplier * xCub + (1 - stearingMultiplier) * rawX;
          double rotOut = stearingMultiplier * rCub + (1 - stearingMultiplier) * rawRot;

          return drive
              .withVelocityX(yOut * MaxSpeed)
              .withVelocityY(xOut * MaxSpeed)
              .withRotationalRate(rotOut * MaxAngularRate);
        }));

    drivetrain.configureAutoBuilder();

    // Button commands
    joystick.button(Constants.Joystick.strafeLeft).onTrue(goToLastSeenTag(TagSide.LEFT, 0.164338, 0.355)); // 0.44
    joystick.button(Constants.Joystick.strafeRight).onTrue(goToLastSeenTag(TagSide.RIGHT, 0.164338, 0.355)); // 0.44

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
