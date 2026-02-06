// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Contains field element locations for the 2026 REBUILT game.
 *
 * <p>All constants are defined from the perspective of the blue alliance.
 */
public class FieldConstants {

  // Load the official field layout
  private static final AprilTagFieldLayout aprilTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // Field dimensions
  public static final double fieldLength = aprilTagLayout.getFieldLength();
  public static final double fieldWidth = aprilTagLayout.getFieldWidth();

  /** Hub related constants */
  public static class Hub {
    public static final double width = Units.inchesToMeters(47.0);

    // Blue alliance hub center (X position from AprilTag 26 + half hub width)
    public static final Translation2d blueCenter =
        new Translation2d(
            aprilTagLayout.getTagPose(26).get().getX() + width / 2.0, fieldWidth / 2.0);

    // Red alliance hub center (X position from AprilTag 4 + half hub width)
    public static final Translation2d redCenter =
        new Translation2d(
            aprilTagLayout.getTagPose(4).get().getX() + width / 2.0, fieldWidth / 2.0);
  }

  /** Zone boundaries (X coordinates) */
  public static class Zones {
    // Blue alliance zone boundary (from blue wall to this X)
    public static final double blueAllianceZoneX =
        aprilTagLayout.getTagPose(26).get().getX() + Hub.width;

    // Neutral zone boundaries
    public static final double neutralZoneStartX = fieldLength / 2.0 - Units.inchesToMeters(120);
    public static final double neutralZoneEndX = fieldLength / 2.0 + Units.inchesToMeters(120);

    // Red alliance zone boundary (from this X to red wall)
    public static final double redAllianceZoneX =
        aprilTagLayout.getTagPose(4).get().getX() - Hub.width;
  }

  /** Calculate passing target based on robot position and alliance */
  public static Translation2d getPassingTarget(Translation2d robotPosition, boolean isRedAlliance) {
    // Passing target X: center of our alliance zone
    double passingX;
    if (isRedAlliance) {
      // Red alliance zone is from redAllianceZoneX to fieldLength
      passingX = (Zones.redAllianceZoneX + fieldLength) / 2.0;
    } else {
      // Blue alliance zone is from 0 to blueAllianceZoneX
      passingX = Zones.blueAllianceZoneX / 2.0;
    }

    // Passing target Y: center of top 1/3 or bottom 1/3 depending on robot Y position
    double passingY;
    if (robotPosition.getY() > fieldWidth / 2.0) {
      // Robot is in top half - target center of top 1/3
      passingY = fieldWidth * (5.0 / 6.0); // Center of top 1/3
    } else {
      // Robot is in bottom half - target center of bottom 1/3
      passingY = fieldWidth * (1.0 / 6.0); // Center of bottom 1/3
    }

    return new Translation2d(passingX, passingY);
  }

  /** Determine if robot is in its own alliance zone */
  public static boolean isInOwnAllianceZone(Translation2d robotPosition, boolean isRedAlliance) {
    if (isRedAlliance) {
      return robotPosition.getX() > Zones.redAllianceZoneX;
    } else {
      return robotPosition.getX() < Zones.blueAllianceZoneX;
    }
  }

  /** Get the hub center for the specified alliance */
  public static Translation2d getHubCenter(boolean isRedAlliance) {
    return isRedAlliance ? Hub.redCenter : Hub.blueCenter;
  }
}
