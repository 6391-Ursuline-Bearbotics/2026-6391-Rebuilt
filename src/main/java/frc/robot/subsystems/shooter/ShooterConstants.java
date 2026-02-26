package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

public final class ShooterConstants {
  // CAN IDs
  public static final int leftMotorId = 7;
  public static final int rightMotorId = 8;
  public static final CANBus canBus = new CANBus("");

  // Motor configuration
  public static final double gearRatio = 1.0;
  public static final boolean leftMotorInverted = true;
  public static final boolean rightMotorInverted = false; // Mirrors left
  public static final double statorCurrentLimit = 80.0;
  public static final Slot0Configs gains = new Slot0Configs().withKP(0.1).withKV(0.12).withKS(0.0);

  // Velocity setpoints
  public static final double idleRPM = 0.0;
  public static final double ejectRPM = -2000.0;

  // Setpoint tolerance for isAtSetpoint()
  public static final double toleranceRPM = 100.0;

  // Shoot-on-the-move
  public static final int shotCompensationIterations = 3;

  // Hood servos (PWM ports - placeholder, update when wired)
  public static final int hoodLeftServoPWM = 0;
  public static final int hoodRightServoPWM = 1;
  public static final double hoodMinAngleDeg = 20.0;
  public static final double hoodMaxAngleDeg = 45.0;

  // Simulation
  public static final double simMOI = 0.001;

  /**
   * Distance (meters) -> Shooter RPM interpolation table. Populate with empirical data from
   * testing. Values below are placeholders.
   */
  public static InterpolatingDoubleTreeMap createDistanceToRPMMap() {
    var map = new InterpolatingDoubleTreeMap();
    // TODO: Replace with real measured data
    map.put(1.0, 3000.0);
    map.put(2.0, 3500.0);
    map.put(3.0, 4000.0);
    map.put(4.0, 4500.0);
    map.put(5.0, 5000.0);
    map.put(6.0, 5500.0);
    map.put(7.0, 6000.0);
    return map;
  }

  /**
   * Distance (meters) -> Hood angle (degrees) interpolation table. Populate with empirical data
   * from testing. Values below are placeholders.
   */
  public static InterpolatingDoubleTreeMap createDistanceToAngleMap() {
    var map = new InterpolatingDoubleTreeMap();
    // TODO: Replace with real measured data
    map.put(1.0, 20.0);
    map.put(2.0, 23.0);
    map.put(3.0, 27.0);
    map.put(4.0, 31.0);
    map.put(5.0, 36.0);
    map.put(6.0, 40.0);
    map.put(7.0, 45.0);
    return map;
  }

  /**
   * Distance (meters) -> Time of flight (seconds) interpolation table for shoot-on-the-move
   * compensation. Populate with empirical data from testing. Values below are placeholders.
   */
  public static InterpolatingDoubleTreeMap createDistanceToTOFMap() {
    var map = new InterpolatingDoubleTreeMap();
    // TODO: Replace with real measured data
    map.put(1.0, 0.25);
    map.put(2.0, 0.35);
    map.put(3.0, 0.50);
    map.put(4.0, 0.65);
    map.put(5.0, 0.80);
    map.put(6.0, 0.95);
    map.put(7.0, 1.10);
    return map;
  }
}
