// src/main/java/frc/robot/util/TagUtils.java
package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Logger;
import frc.robot.RobotContainer;

public class TagUtils {
  // Pull in your field’s AprilTag layout
  private static final AprilTagFieldLayout kTagLayout = Constants.Vision.kTagLayout;

  /** @return 2D pose of that AprilTag, if it exists in the layout */
  public static Optional<Pose2d> getTagPose2d(int tagId) {
    Logger.warn("Tag Id {}", tagId);
    return kTagLayout
        .getTagPose(tagId)
        .map(p3 -> new Pose2d(p3.getX(), p3.getY(), p3.getRotation().toRotation2d()));
  }

  /**
   * Compute a goal Pose2d offset from the tag.
   * 
   * @param tagId             AprilTag ID
   * @param side              Which side to approach on
   * @param offsetMeters      how far left or right the tag (in meters)
   * @param frontoffsetMeters How far off the reef
   */

  public static Pose2d computeTagAdjacencyPose(int tagId, TagSide side, double offsetMeters, double frontoffsetMeters) {
    Pose2d tagPose = getTagPose2d(tagId)
        .orElse(RobotContainer.drivetrain.getPose());
    // Tag 6
    if (tagId == 6) {
      Translation2d override = new Translation2d(0, 0);
      Translation2d front = new Translation2d(0, 0);
      if (side == TagSide.LEFT) {
        override = new Translation2d(-offsetMeters * (Math.sqrt(3) / 2), -offsetMeters / 2);
      } else /* assume RIGHT */ {
        override = new Translation2d(+offsetMeters * (Math.sqrt(3) / 2), +offsetMeters / 2);
      }
      front = new Translation2d(+frontoffsetMeters / Math.sqrt(3), -frontoffsetMeters * 2 / Math.sqrt(3));
      return new Pose2d(
          tagPose.getTranslation().plus(override).plus(front),
          tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }
    // Tag 7
    else if (tagId == 7) {
      Translation2d override = new Translation2d(0, 0);
      Translation2d front = new Translation2d(0, 0);
      if (side == TagSide.LEFT) {
        override = new Translation2d(0, -offsetMeters);
      } else /* assume RIGHT */ {
        override = new Translation2d(0, +offsetMeters);
      }
      front = new Translation2d(+frontoffsetMeters, 0);
      return new Pose2d(
          tagPose.getTranslation().plus(override).plus(front),
          tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    // Tag 8
    else if (tagId == 8) {
      Translation2d override = new Translation2d(0, 0);
      Translation2d front = new Translation2d(0, 0);
      if (side == TagSide.LEFT) {
        override = new Translation2d(+offsetMeters * (Math.sqrt(3) / 2), -offsetMeters / 2);
      } else /* assume RIGHT */ {
        override = new Translation2d(-offsetMeters * (Math.sqrt(3) / 2), +offsetMeters / 2);
      }
      front = new Translation2d(+frontoffsetMeters / Math.sqrt(3), +frontoffsetMeters * 2 / Math.sqrt(3));
      return new Pose2d(
          tagPose.getTranslation().plus(override).plus(front),
          tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    // Tag 9
    else if (tagId == 9) {
      Translation2d override = new Translation2d(0, 0);
      Translation2d front = new Translation2d(0, 0);
      if (side == TagSide.LEFT) {
        override = new Translation2d(+offsetMeters * (Math.sqrt(3) / 2), +offsetMeters / 2);
      } else /* assume RIGHT */ {
        override = new Translation2d(-offsetMeters * (Math.sqrt(3) / 2), -offsetMeters / 2);
      }
      front = new Translation2d(-frontoffsetMeters / Math.sqrt(3), +frontoffsetMeters * 2 / Math.sqrt(3));
      return new Pose2d(
          tagPose.getTranslation().plus(override).plus(front),
          tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    // Tag 10
    else if (tagId == 10) {
      Translation2d override = new Translation2d(0, 0);
      Translation2d front = new Translation2d(0, 0);
      if (side == TagSide.LEFT) {
        override = new Translation2d(0, +offsetMeters);
      } else /* assume RIGHT */ {
        override = new Translation2d(0, -offsetMeters);
      }
      front = new Translation2d(-frontoffsetMeters,0);
      return new Pose2d(
          tagPose.getTranslation().plus(override).plus(front),
          tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    // Tag 11
    else if (tagId == 11) {
      Translation2d override = new Translation2d(0, 0);
      Translation2d front = new Translation2d(0, 0);
      if (side == TagSide.LEFT) {
        override = new Translation2d(-offsetMeters * (Math.sqrt(3) / 2), +offsetMeters / 2);
      } else /* assume RIGHT */ {
        override = new Translation2d(+offsetMeters * (Math.sqrt(3) / 2), -offsetMeters / 2);
      }
      front = new Translation2d(-frontoffsetMeters / Math.sqrt(3), -frontoffsetMeters * 2 / Math.sqrt(3));
      return new Pose2d(
          tagPose.getTranslation().plus(override).plus(front),
          tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    // Tag 17
    else if (tagId == 17) {
      Translation2d override = new Translation2d(0, 0);
      Translation2d front = new Translation2d(0, 0);
      if (side == TagSide.LEFT) {
        override = new Translation2d(-offsetMeters * (Math.sqrt(3) / 2), +offsetMeters / 2);
      } else /* assume RIGHT */ {
        override = new Translation2d(+offsetMeters * (Math.sqrt(3) / 2), -offsetMeters / 2);
      }
      front = new Translation2d(-frontoffsetMeters / Math.sqrt(3), -frontoffsetMeters * 2 / Math.sqrt(3));
      return new Pose2d(
          tagPose.getTranslation().plus(override).plus(front),
          tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    // Tag 18
    else if (tagId == 18) {
      Translation2d override = new Translation2d(0, 0);
      Translation2d front = new Translation2d(0, 0);
      if (side == TagSide.LEFT) {
        override = new Translation2d(0, +offsetMeters);
      } else /* assume RIGHT */ {
        override = new Translation2d(0, -offsetMeters);
      }
      front = new Translation2d(-frontoffsetMeters, 0);
      return new Pose2d(
          tagPose.getTranslation().plus(override).plus(front),
          tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    // Tag 19
    else if (tagId == 19) {
      Translation2d override = new Translation2d(0, 0);
      Translation2d front = new Translation2d(0, 0);
      if (side == TagSide.LEFT) {
        override = new Translation2d(+offsetMeters * (Math.sqrt(3) / 2), +offsetMeters / 2);
      } else /* assume RIGHT */ {
        override = new Translation2d(-offsetMeters * (Math.sqrt(3) / 2), -offsetMeters / 2);
      }
      front = new Translation2d(-frontoffsetMeters / Math.sqrt(3), +frontoffsetMeters * 2 / Math.sqrt(3));
      return new Pose2d(
          tagPose.getTranslation().plus(override).plus(front),
          tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    // Tag 20
    else if (tagId == 20) {
      Translation2d override = new Translation2d(0, 0);
      Translation2d front = new Translation2d(0, 0);
      if (side == TagSide.LEFT) {
        override = new Translation2d(+offsetMeters * (Math.sqrt(3) / 2), -offsetMeters / 2);
      } else /* assume RIGHT */ {
        override = new Translation2d(-offsetMeters * (Math.sqrt(3) / 2), +offsetMeters / 2);
      }
      front = new Translation2d(+frontoffsetMeters / Math.sqrt(3), +frontoffsetMeters * 2 / Math.sqrt(3));
      return new Pose2d(
          tagPose.getTranslation().plus(override).plus(front),
          tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));
    }

    // Tag 21
    else if (tagId == 21) {
      Translation2d override = new Translation2d(0, 0);
      Translation2d front = new Translation2d(0, 0);
      if (side == TagSide.LEFT) {
        override = new Translation2d(0, -offsetMeters);
      } else /* assume RIGHT */ {
        override = new Translation2d(0, +offsetMeters);
      }
      front = new Translation2d(+frontoffsetMeters, 0);
      return new Pose2d(
          tagPose.getTranslation().plus(override).plus(front),
          tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));

    }

    // Tag 22
    else if (tagId == 22) {
      Translation2d override = new Translation2d(0, 0);
      Translation2d front = new Translation2d(0, 0);
      if (side == TagSide.LEFT) {
        override = new Translation2d(-offsetMeters * (Math.sqrt(3) / 2), -offsetMeters / 2);
      } else /* assume RIGHT */ {
        override = new Translation2d(+offsetMeters * (Math.sqrt(3) / 2), +offsetMeters / 2);
      }
      
      return new Pose2d(
          tagPose.getTranslation().plus(override).plus(front),
          tagPose.getRotation().plus(Rotation2d.fromDegrees(180)));

    } else {
      Logger.warn("No valid tag was found");
    }
    return tagPose;

  }
};

// rotate that local offset into field frame
