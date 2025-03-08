// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MusicTone;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathPlannerPath;

public class RobotContainer {
    public static double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 1; // kSpeedAt12Volts desired top speed
    public static double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.01).withRotationalDeadband(MaxAngularRate * 0.01) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandJoystick joystick = new CommandJoystick(0);
    private final XboxController Xbox = new XboxController(1);

    // private final TalonFX frontLeftDrive = new TalonFX(1);
    // private final TalonFX frontLeftSteer = new TalonFX(2);
    // private final TalonFX frontRightDrive = new TalonFX(3);
    // private final TalonFX frontRightSteer = new TalonFX(4);
    // private final TalonFX backLeftDrive = new TalonFX(5);
    // private final TalonFX backLeftSteer = new TalonFX(6);
    // private final TalonFX backRightDrive = new TalonFX(7);
    // private final TalonFX backRightSteer = new TalonFX(8 );
    // private final TalonFX liftLeft = new TalonFX(10, "rio");
    // private final TalonFX liftRight = new TalonFX(11, "rio");

    final DutyCycleOut m_leftRequest = new DutyCycleOut(0.0);
    final DutyCycleOut m_rightRequest = new DutyCycleOut(0.0);



    Orchestra m_Orchestra = new Orchestra();

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final SendableChooser<Command> autoChooser;
    public RobotContainer() {
        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser();

        SmartDashboard.putData("Auto mode", autoChooser);
    }

    private void configureBindings() {
        
        
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
                drive.withVelocityX(-joystick.getX() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getY() * MaxSpeed) // Drive left with negative X (left)
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

    // public void configureOrchestra() {
    //     m_Orchestra.addInstrument(frontLeftDrive, 1);
    //     m_Orchestra.addInstrument(frontLeftSteer, 2);
    //     m_Orchestra.addInstrument(frontRightDrive, 3);
    //     m_Orchestra.addInstrument(frontRightSteer, 4);
    //     m_Orchestra.addInstrument(backLeftDrive, 5);
    //     m_Orchestra.addInstrument(backLeftSteer, 6);
    //     m_Orchestra.addInstrument(backRightDrive, 7);
    //     m_Orchestra.addInstrument(backRightSteer, 8);
    //     var status = m_Orchestra.loadMusic("SkyOfTrees.chrp");
    //     Commands.print("Configuring orchestra...");
    //     if (status.isOK()) {
    //         Commands.print("Orchestra configured");
    //      }
    // }

    public Command getAutonomousCommand() {
        try{
        // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Blue 1");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
    }
}
