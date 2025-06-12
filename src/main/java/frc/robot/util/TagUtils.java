package frc.robot.util;

import java.util.Optional;
import java.util.Map;
import static java.util.Map.entry;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Logger;
import frc.robot.RobotContainer;

public class TagUtils {
  private static class TagOffsetConfig {
    final Translation2d leftDir;
    final Translation2d rightDir;
    final Translation2d frontDir;
    TagOffsetConfig(Translation2d leftDir, Translation2d rightDir, Translation2d frontDir) {
      this.leftDir = leftDir;
      this.rightDir = rightDir;
      this.frontDir = frontDir;
    }
  }

  private static final Map<Integer, TagOffsetConfig> kTagConfigs = Map.ofEntries(
    entry(6,  new TagOffsetConfig(
      new Translation2d(-Math.sqrt(3)/2, -0.5),
      new Translation2d( Math.sqrt(3)/2,  0.5),
      new Translation2d( 1/Math.sqrt(3), -2/Math.sqrt(3))
    )),
    entry(7,  new TagOffsetConfig(
      new Translation2d( 0, -1),
      new Translation2d( 0,  1),
      new Translation2d( 1,  0)
    )),
    entry(8, new TagOffsetConfig(
      new Translation2d( Math.sqrt(3)/2, -0.5),
      new Translation2d(-Math.sqrt(3)/2,  0.5),
      new Translation2d( 1/Math.sqrt(3),  2/Math.sqrt(3))
    )),
    entry(9, new TagOffsetConfig(
      new Translation2d( Math.sqrt(3)/2,  0.5),
      new Translation2d(-Math.sqrt(3)/2, -0.5),
      new Translation2d(-1/Math.sqrt(3),  2/Math.sqrt(3))
    )),
    entry(10, new TagOffsetConfig(
      new Translation2d( 0,  1),
      new Translation2d( 0, -1),
      new Translation2d(-1,  0) // Adjust front offset with -0.107 removed
    )),
    entry(11, new TagOffsetConfig(
      new Translation2d(-Math.sqrt(3)/2,  0.5),
      new Translation2d( Math.sqrt(3)/2, -0.5),
      new Translation2d(-1/Math.sqrt(3), -2/Math.sqrt(3))
    )),
    entry(17, new TagOffsetConfig(
      new Translation2d(-Math.sqrt(3)/2,  0.5),
      new Translation2d( Math.sqrt(3)/2, -0.5),
      new Translation2d(-1/Math.sqrt(3), -2/Math.sqrt(3))
    )),
    entry(18, new TagOffsetConfig(
      new Translation2d( 0,  1),
      new Translation2d( 0, -1),
      new Translation2d(-1,  0)
    )),
    entry(19, new TagOffsetConfig(
      new Translation2d( Math.sqrt(3)/2,  0.5),
      new Translation2d(-Math.sqrt(3)/2, -0.5),
      new Translation2d(-1/Math.sqrt(3),  2/Math.sqrt(3))
    )),
    entry(20, new TagOffsetConfig(
      new Translation2d( Math.sqrt(3)/2, -0.5),
      new Translation2d(-Math.sqrt(3)/2,  0.5),
      new Translation2d( 1/Math.sqrt(3),  2/Math.sqrt(3))
    )),
    entry(21, new TagOffsetConfig(
      new Translation2d( 0, -1),
      new Translation2d( 0,  1),
      new Translation2d( 1,  0)
    )),
    entry(22, new TagOffsetConfig(
      new Translation2d(-Math.sqrt(3)/2, -0.5),
      new Translation2d( Math.sqrt(3)/2,  0.5),
      new Translation2d( 1/Math.sqrt(3), -2/Math.sqrt(3))
    ))
  );

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

  public static Pose2d computeTagAdjacencyPose(int tagId, tagSide side, double offsetMeters, double frontoffsetMeters) {
    Pose2d tagPose = getTagPose2d(tagId)
        .orElse(RobotContainer.drivetrain.getPose());

    TagOffsetConfig cfg = kTagConfigs.get(tagId);
    if (cfg == null) {
      Logger.warn("No valid tag config for ID {}", tagId);
      return tagPose;
    }
    Translation2d dir = (side == tagSide.LEFT) ? cfg.leftDir : cfg.rightDir;
    Translation2d override = dir.times(offsetMeters);
    Translation2d front   = cfg.frontDir.times(frontoffsetMeters);
    return new Pose2d(
      tagPose.getTranslation().plus(override).plus(front),
      tagPose.getRotation().plus(Rotation2d.fromDegrees(180))
    );

  }
};

// rotate that local offset into field frame