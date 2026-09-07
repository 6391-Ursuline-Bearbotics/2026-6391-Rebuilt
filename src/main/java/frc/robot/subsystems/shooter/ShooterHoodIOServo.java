package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

public class ShooterHoodIOServo implements ShooterHoodIO {
  private final Servo leftServo;
  private final Servo rightServo;

  private double targetAngleDeg = ShooterConstants.hoodMinAngleDeg;
  private double estimatedAngleDeg = ShooterConstants.hoodMinAngleDeg;
  private double lastUpdateTimestamp = Timer.getFPGATimestamp();

  public ShooterHoodIOServo() {
    leftServo = new Servo(ShooterConstants.hoodLeftServoPWM);
    rightServo = new Servo(ShooterConstants.hoodRightServoPWM);
  }

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    updateEstimate();
    inputs.positionDeg = estimatedAngleDeg;
  }

  @Override
  public void setAngle(double angleDeg) {
    updateEstimate();
    targetAngleDeg =
        MathUtil.clamp(
            angleDeg, ShooterConstants.hoodMinAngleDeg, ShooterConstants.hoodMaxAngleDeg);
    double servoPosition =
        (targetAngleDeg - ShooterConstants.hoodMinAngleDeg)
            / (ShooterConstants.hoodMaxAngleDeg - ShooterConstants.hoodMinAngleDeg);
    leftServo.set(servoPosition);
    rightServo.set(servoPosition);
  }

  @Override
  public void stop() {
    setAngle(ShooterConstants.hoodMinAngleDeg);
  }

  private void updateEstimate() {
    double now = Timer.getFPGATimestamp();
    double maxDelta =
        ShooterConstants.hoodMaxVelocityDegPerSec * Math.max(0.0, now - lastUpdateTimestamp);
    estimatedAngleDeg += MathUtil.clamp(targetAngleDeg - estimatedAngleDeg, -maxDelta, maxDelta);
    lastUpdateTimestamp = now;
  }
}
