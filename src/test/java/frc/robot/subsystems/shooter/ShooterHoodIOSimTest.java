package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class ShooterHoodIOSimTest {
  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void reportsMotionOverTimeInsteadOfInstantlyMirroringCommand() {
    ShooterHoodIOSim io = new ShooterHoodIOSim();
    ShooterHoodIOInputsAutoLogged inputs = new ShooterHoodIOInputsAutoLogged();
    io.setAngle(ShooterConstants.hoodMaxAngleDeg);

    io.updateInputs(inputs);
    assertTrue(inputs.positionDeg > ShooterConstants.hoodMinAngleDeg);
    assertTrue(inputs.positionDeg < ShooterConstants.hoodMaxAngleDeg);

    for (int i = 0; i < 100; i++) {
      io.updateInputs(inputs);
    }
    assertEquals(ShooterConstants.hoodMaxAngleDeg, inputs.positionDeg, 1e-9);
  }
}
