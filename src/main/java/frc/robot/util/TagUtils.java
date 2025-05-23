// src/main/java/frc/robot/util/TagUtils.java
package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;

public class TagUtils {
  // Pull in your field’s AprilTag layout
  private static final AprilTagFieldLayout kTagLayout = Constants.Vision.kTagLayout;

  /** @return 2D pose of that AprilTag, if it exists in the layout */
  public static Optional<Pose2d> getTagPose2d(int tagId) {
    return kTagLayout
      .getTagPose(tagId)
      .map(p3 -> new Pose2d(p3.getX(), p3.getY(), p3.getRotation().toRotation2d()));
  }

  /**
   * Compute a goal Pose2d offset from the tag.
   * @param tagId AprilTag ID
   * @param side  Which side to approach on
   * @param offsetMeters how far off the tag (in meters)
   */
  public static Pose2d computeTagAdjacencyPose(
      int tagId,
      TagSide side,
      double offsetMeters
  ) {
    Pose2d tag = getTagPose2d(tagId)
      .orElseThrow(() -> new IllegalArgumentException("Bad AprilTag ID: " + tagId));

    Translation2d local;
    switch (side) {
      case LEFT:    local = new Translation2d( 0, +offsetMeters); break;
      case RIGHT:   local = new Translation2d( 0, -offsetMeters); break;
      case INWARD:  local = new Translation2d(+offsetMeters,  0); break;
      case OUTWARD: local = new Translation2d(-offsetMeters,  0); break;
      default:      local = new Translation2d(0,0);             break;
    }

    // rotate that local offset into field frame
    Translation2d fieldOffset = local.rotateBy(tag.getRotation());    // face back to the tag
    Rotation2d goalYaw = tag.getRotation().plus(Rotation2d.fromDegrees(180));

    return new Pose2d(tag.getTranslation().plus(fieldOffset), goalYaw);
  }


}