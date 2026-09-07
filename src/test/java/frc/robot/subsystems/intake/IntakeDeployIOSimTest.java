package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

class IntakeDeployIOSimTest {
  @BeforeAll
  static void initializeHal() {
    assertTrue(HAL.initialize(500, 0));
  }

  @Test
  void retractedHardStopExceedsRetractThreshold() {
    IntakeDeployIOSim io = new IntakeDeployIOSim();
    IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();

    io.setVoltage(IntakeConstants.retractVoltage);
    io.updateInputs(inputs);

    assertTrue(inputs.statorCurrentAmps > IntakeConstants.retractCurrentThreshold);
  }
}
