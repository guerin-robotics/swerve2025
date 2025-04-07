package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.MiniPID;

public class Vision extends SubsystemBase {
  private static Timer yTimer = new Timer();
  public static int target = 0;
  public static double limelight_aim_proportional(double MaxAngularSpeed, int position, String name) {
    // MiniPID aimPID = new MiniPID(0.1, 0.0, 10); 

    Double targetOffset = 0.0;
  if (position == 1) {
    targetOffset = 20.0;
  }
  else if (position == 2) {
    targetOffset = -20.0;
  }
  double kP = 0.005;
  // Timer.delay(0.01);
  double targetingAngularVelocity = 0;
  if (LimelightHelpers.getTA(name) > 0) {
    targetingAngularVelocity = (LimelightHelpers.getTX(name) + targetOffset) * kP;
  }

  targetingAngularVelocity *= MaxAngularSpeed;

  //invert since tx is positive when the target is to the right of the crosshair
  targetingAngularVelocity *= -1.0;

  return targetingAngularVelocity;
  }
    
      // simple proportional ranging control with Limelight's "ty" value
      // this works best if your Limelight's mount height and target mount height are different.
      // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public static double limelight_range_proportional(double MaxAngularSpeed, String name) {   
    // MiniPID rangePID = new MiniPID(0.5, 0.0, 10);
    double kP = -0.01;
    double targetingForwardSpeed = LimelightHelpers.getTY(name) * kP;
    // System.out.println(targetingForwardSpeed);
    targetingForwardSpeed *= MaxAngularSpeed;
    targetingForwardSpeed *= 1 * Constants.masterSpeedMultiplier;
    // Timer.delay(0.01);
    return targetingForwardSpeed;
  }
  public static double alignRobot(double MaxAngularSpeed, double angle) {
    double kP = 0.6;
    // angle *= Math.PI/180;
    System.out.println("Target angle: " + RobotContainer.drivetrain.getRotation3d().getZ() + angle);
    double alignSpeed = (RobotContainer.drivetrain.getRotation3d().getZ() + angle)%360 * kP;

    alignSpeed *= MaxAngularSpeed;
    alignSpeed *= -1 * Constants.masterSpeedMultiplier;
    // System.out.println(alignSpeed);

    return alignSpeed;
  }
  
    static double xSpeed;
    static double rot;

    // public static double yAlign(double MaxAngularSpeed) {
    //   double kP = 0.1;

    // }

    // public static void strafe(boolean side) {
    //   yTimer.start();
    //   while (yTimer.get() < 1) {
    //     if (side == true) {
    //       RobotContainer.drivetrain.setControl(RobotContainer.drive.withVelocityX(0).withVelocityY(1).withRotationalRate(0));
    //     }
    //     else {
    //       RobotContainer.drivetrain.setControl(RobotContainer.drive.withVelocityX(0).withVelocityY(-1).withRotationalRate(0));
    //     }
    //   }
    // }

    public static void limelight_align(double MaxAngularRate, boolean side) {
      double rotation = rotateToTarget(MaxAngularRate, "limelight-right");
      System.out.println(rotation);
      double kP = 30;
      double alignSpeed = ((LimelightHelpers.getTX("limelight-right") + Math.atan(7/6)) * (Math.PI/180)) * kP;
      // double alignSpeed = 1;
      System.out.println("Aligning at speed: " + alignSpeed);
      if (side == true) {
        System.out.println("Driving right");
        RobotContainer.drivetrain.setControl(RobotContainer.drive.withVelocityX(0).withVelocityY(-alignSpeed).withRotationalRate(0));
      }
      else {
        RobotContainer.drivetrain.setControl(RobotContainer.drive.withVelocityX(0).withVelocityY(alignSpeed).withRotationalRate(0));
      }
    }

  public static void applyLimelight(double MaxAngularRate, int Position, String name) {
    int target = (int) LimelightHelpers.getFiducialID(name);
    // System.out.println("Current tag: " + LimelightHelpers.getFiducialID(name) + ", ID read: " + target);
    var angle = -1;
    switch (target) {
      case 6:
        angle = 120; //300 normally
        break;
      case 7:
        angle = 180;
        break;
      case 8:
        angle = 240;
        break;
      case 9:
        angle = 300;
        break;
      case 10:
        angle = 0;
        break;
      case 11:
        angle = 60;
        break;
      case 17:
        angle = 120;
        break;
      case 18:
        angle = 180;
        break;
      case 19:
        angle = 240;
        break;
      case 20:
        angle = 300;
        break;
      case 21:
        angle = 0;
        break;
      case 22:
        angle = 60;
        break;
    }
  
    final var rot_gyro = alignRobot(MaxAngularRate, angle);
    // System.out.println("Limelight activated");
    final var rot_limelight = limelight_aim_proportional(MaxAngularRate, Position, name);
    rot = rot_limelight * 0.65;
    final var forward_limelight = limelight_range_proportional(MaxAngularRate, name);
    xSpeed = forward_limelight * 0.65 * Constants.masterSpeedMultiplier;
    // System.out.println("Rotation: " + rot_limelight + "Distance: " + forward_limelight);

    // RobotContainer.drivetrain.setDefaultCommand(RobotContainer.drivetrain.applyRequest(() -> RobotContainer.drive.withVelocityX(xSpeed).withVelocityY(0).withRotationalRate(rot)));
    RobotContainer.drivetrain.setControl(RobotContainer.drive.withVelocityX(xSpeed).withVelocityY(rot_limelight).withRotationalRate(rot_gyro));
  }

  public static void setTarget(String name) {
    target = (int) LimelightHelpers.getFiducialID(name);
    System.out.println(target);
  }

  private static double rotateToTarget(double MaxAngularRate, String name) {
    // int target = (int) LimelightHelpers.getFiducialID(name);
    var angle = 0;
    switch (target) {
      case 6:
        angle = 120; //300 normally
        break;
      case 7:
        angle = 180;
        break;
      case 8:
        angle = 240;
        break;
      case 9:
        angle = 300;
        break;
      case 10:
        angle = 0;
        break;
      case 11:
        angle = 60;
        break;
      case 17:
        angle = 120;
        break;
      case 18:
        angle = 180;
        break;
      case 19:
        angle = 240;
        break;
      case 20:
        angle = 300;
        break;
      case 21:
        angle = 0;
        break;
      case 22:
        angle = 60;
        break;
    }
    final var rot_gyro = alignRobot(MaxAngularRate, angle);
    return rot_gyro;
  }
}
