package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class LEDSubsystem extends SubsystemBase {
  private final AddressableLED m_led;
  private final AddressableLEDBuffer m_ledBuffer;
  private final BooleanSupplier m_isHoodDownSupplier;

  public LEDSubsystem(int pwmPort, int length, BooleanSupplier isHoodDownSupplier) {
    m_isHoodDownSupplier = isHoodDownSupplier;

    if (Constants.currentMode != Constants.Mode.REPLAY) {
      m_led = new AddressableLED(pwmPort);
      m_ledBuffer = new AddressableLEDBuffer(length);
      m_led.setLength(m_ledBuffer.getLength());
      m_led.setData(m_ledBuffer);
      m_led.start();
    } else {
      m_led = null;
      m_ledBuffer = null;
    }
  }

  @Override
  public void periodic() {
    boolean hoodDown = m_isHoodDownSupplier.getAsBoolean();
    Logger.recordOutput("LED/HoodDown", hoodDown);
    Logger.recordOutput("LED/Color", hoodDown ? "Green" : "Red");

    if (m_led != null && m_ledBuffer != null) {
      if (hoodDown) {
        // Green when Hood is down / stowed
        setRGB(0, 255, 0);
      } else {
        // Red when Hood is deployed
        setRGB(255, 0, 0);
      }
      m_led.setData(m_ledBuffer);
    }
  }

  private void setRGB(int r, int g, int b) {
    if (m_ledBuffer == null) {
      return;
    }
    for (int i = 0; i < m_ledBuffer.getLength(); i++) {
      m_ledBuffer.setRGB(i, r, g, b);
    }
  }
}
