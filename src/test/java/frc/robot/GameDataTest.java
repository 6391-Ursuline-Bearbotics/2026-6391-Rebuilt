package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import org.junit.jupiter.api.Test;

class GameDataTest {
  @Test
  void followsEachTeleopShiftForRedInactiveFirst() {
    assertTrue(GameData.isHubActiveInTeleop(Alliance.Red, 131.0, "R"));
    assertFalse(GameData.isHubActiveInTeleop(Alliance.Red, 120.0, "R"));
    assertTrue(GameData.isHubActiveInTeleop(Alliance.Red, 95.0, "R"));
    assertFalse(GameData.isHubActiveInTeleop(Alliance.Red, 70.0, "R"));
    assertTrue(GameData.isHubActiveInTeleop(Alliance.Red, 45.0, "R"));
    assertTrue(GameData.isHubActiveInTeleop(Alliance.Red, 25.0, "R"));
  }

  @Test
  void usesCurrentMessageRatherThanPreviousMatchMessage() {
    assertFalse(GameData.isHubActiveInTeleop(Alliance.Red, 120.0, "R"));
    assertTrue(GameData.isHubActiveInTeleop(Alliance.Red, 120.0, "B"));
  }

  @Test
  void failsOpenForMissingOrMalformedData() {
    assertTrue(GameData.isHubActiveInTeleop(Alliance.Blue, 120.0, ""));
    assertTrue(GameData.isHubActiveInTeleop(Alliance.Blue, 120.0, null));
    assertTrue(GameData.isHubActiveInTeleop(Alliance.Blue, 120.0, "?"));
  }
}
