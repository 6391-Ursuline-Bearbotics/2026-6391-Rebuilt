package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;

public final class IndexerConstants {
  // CAN IDs
  public static final int beltMotorId = 5;
  public static final int kickerMotorId = 6;
  public static final CANBus canBus = new CANBus("");

  // Belt motor
  public static final double beltGearRatio = 1.0;
  public static final boolean beltMotorInverted = true;
  public static final double beltStatorCurrentLimit = 80.0;
  public static final double beltFeedVelocityRPM = 2000.0;
  public static final double beltEjectVelocityRPM = -2000.0;
  public static final Slot0Configs beltGains =
      new Slot0Configs().withKP(0.1).withKV(0.12).withKS(0.0);

  // Kicker motor
  public static final double kickerGearRatio = 1.0;
  public static final boolean kickerMotorInverted = true;
  public static final double kickerStatorCurrentLimit = 80.0;
  public static final double kickerFeedVelocityRPM = 2000.0;
  public static final double kickerEjectVelocityRPM = -2000.0;
  public static final Slot0Configs kickerGains =
      new Slot0Configs().withKP(0.1).withKV(0.12).withKS(0.0);

  // Jam detection
  public static final double jamCurrentThreshold = 40.0; // Stator amps
  public static final double jamDebounceTime = 0.5; // Seconds
  public static final double jamReverseTime = 0.25; // Seconds to reverse

  // Simulation
  public static final double beltSimMOI = 0.001;
  public static final double kickerSimMOI = 0.001;
}
