package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Vision extends SubsystemBase {
  
      public static double limelight_aim_proportional(double MaxAngularSpeed)
    {    
      // kP (constant of proportionality)
      // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
      // if it is too high, the robot will oscillate around.
      // if it is too low, the robot will never reach its target
      // if the robot never turns in the correct direction, kP should be inverted.
      double kP = 0.04;
  
      // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
      // your limelight 3 feed, tx should return roughly 31 degrees.
      double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;
      // System.out.println(LimelightHelpers.getFiducialID("limelight"));
  
      // convert to radians per second for our drive method
      targetingAngularVelocity *= MaxAngularSpeed;
  
      //invert since tx is positive when the target is to the right of the crosshair
      targetingAngularVelocity *= -1.0;
  
      return targetingAngularVelocity;
    }
  
    // simple proportional ranging control with Limelight's "ty" value
    // this works best if your Limelight's mount height and target mount height are different.
    // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
    public static double limelight_range_proportional(double MaxAngularSpeed) {    
      double kP = 0.03;
      double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
      targetingForwardSpeed *= MaxAngularSpeed;
      targetingForwardSpeed *= -1;
      return targetingForwardSpeed;
    }
  
    public static double alignRobot(double MaxAngularSpeed) {
      double kP = 0.8;
      double arbitraryAngle = 60;
      arbitraryAngle *= Math.PI/180;
      double alignSpeed = (RobotContainer.drivetrain.getRotation3d().getZ() + arbitraryAngle)%(360+arbitraryAngle) * kP;

      alignSpeed *= MaxAngularSpeed;
      alignSpeed *= -1;
      System.out.println(alignSpeed);

      return alignSpeed;
    }

    static double xSpeed;
    static double rot;

  public static void applyLimelight(double MaxAngularRate) {
    final var rot_gyro = alignRobot(MaxAngularRate);
    // System.out.println("Limelight activated");
    final var rot_limelight = limelight_aim_proportional(MaxAngularRate);
    rot = rot_limelight;
    final var forward_limelight = limelight_range_proportional(MaxAngularRate);
    xSpeed = forward_limelight * 0;
    // System.out.println("Rotation: " + rot_limelight + "Distance: " + forward_limelight);

    // RobotContainer.drivetrain.setDefaultCommand(RobotContainer.drivetrain.applyRequest(() -> RobotContainer.drive.withVelocityX(xSpeed).withVelocityY(0).withRotationalRate(rot)));
    RobotContainer.drivetrain.setControl(RobotContainer.drive.withVelocityX(0).withVelocityY(0).withRotationalRate(rot_gyro));
  }
}
