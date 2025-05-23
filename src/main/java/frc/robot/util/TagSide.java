package frc.robot.util;

/** Which side of the tag you want to end up on */
public enum TagSide {
  LEFT,    // +Y in tag‐frame
  RIGHT,   // –Y in tag‐frame
  INWARD,  // +X in tag‐frame (toward field‐origin)
  OUTWARD  // –X in tag‐frame (away from field‐origin)
}