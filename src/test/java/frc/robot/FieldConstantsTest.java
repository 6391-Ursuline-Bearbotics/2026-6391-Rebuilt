package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

class FieldConstantsTest {
  private static final double EPSILON = 1e-9;

  @Test
  void allianceZoneBoundariesMirrorAcrossFieldCenter() {
    assertEquals(
        FieldConstants.fieldLength - FieldConstants.Zones.blueAllianceZoneX,
        FieldConstants.Zones.redAllianceZoneX,
        EPSILON);
  }

  @Test
  void mirroredPositionsHaveMatchingZoneMembership() {
    double blueX = FieldConstants.Zones.blueAllianceZoneX - 0.1;
    double redX = FieldConstants.fieldLength - blueX;
    assertTrue(FieldConstants.isInOwnAllianceZone(new Translation2d(blueX, 1.0), false));
    assertTrue(FieldConstants.isInOwnAllianceZone(new Translation2d(redX, 1.0), true));

    blueX = FieldConstants.Zones.blueAllianceZoneX + 0.1;
    redX = FieldConstants.fieldLength - blueX;
    assertFalse(FieldConstants.isInOwnAllianceZone(new Translation2d(blueX, 1.0), false));
    assertFalse(FieldConstants.isInOwnAllianceZone(new Translation2d(redX, 1.0), true));
  }
}
