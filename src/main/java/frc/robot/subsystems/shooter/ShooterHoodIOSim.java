package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;

public class ShooterHoodIOSim implements ShooterHoodIO {
  private double positionDeg = ShooterConstants.hoodMinAngleDeg;
  private double targetAngleDeg = ShooterConstants.hoodMinAngleDeg;

  @Override
  public void updateInputs(ShooterHoodIOInputs inputs) {
    double maxDelta = ShooterConstants.hoodMaxVelocityDegPerSec * 0.02;
    positionDeg += MathUtil.clamp(targetAngleDeg - positionDeg, -maxDelta, maxDelta);
    inputs.positionDeg = positionDeg;
  }

  @Override
  public void setAngle(double angleDeg) {
    targetAngleDeg =
        MathUtil.clamp(
            angleDeg, ShooterConstants.hoodMinAngleDeg, ShooterConstants.hoodMaxAngleDeg);
  }

  @Override
  public void stop() {
    targetAngleDeg = ShooterConstants.hoodMinAngleDeg;
  }
}
