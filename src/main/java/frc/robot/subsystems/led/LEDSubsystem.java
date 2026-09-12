package frc.robot.subsystems.led;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.BooleanSupplier;

public class LEDSubsystem extends SubsystemBase {
    private final AddressableLED m_led;
    private final AddressableLEDBuffer m_ledBuffer;
    private final BooleanSupplier m_isHoodDownSupplier;

    public LEDSubsystem(int pwmPort, int length, BooleanSupplier isHoodDownSupplier) {
        m_isHoodDownSupplier = isHoodDownSupplier;

        m_led = new AddressableLED(pwmPort);
        m_ledBuffer = new AddressableLEDBuffer(length);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    @Override
    public void periodic() {
        if (m_isHoodDownSupplier.getAsBoolean()) {
            // Green when Hood is down / stowed
            setRGB(0, 255, 0); 
        } else {
            // Red when Hood is deployed
            setRGB(255, 0, 0); 
        }

        m_led.setData(m_ledBuffer);
    }

    private void setRGB(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
    }
}