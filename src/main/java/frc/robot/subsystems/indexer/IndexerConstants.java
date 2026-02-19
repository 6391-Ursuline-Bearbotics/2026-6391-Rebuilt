package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;

public final class IndexerConstants {
  // CAN IDs
  public static final int beltMotorId = 5;
  public static final int kickerMotorId = 5; // for can errors - change back to 6
  public static final CANBus canBus = new CANBus("");

  // Belt motor
  public static final double beltGearRatio = 1.0;
  public static final boolean beltMotorInverted = false;
  public static final double beltStatorCurrentLimit = 80.0;
  public static final double beltFeedVelocityRPM = 3000.0;
  public static final double beltEjectVelocityRPM = -3000.0;
  public static final Slot0Configs beltGains =
      new Slot0Configs().withKP(0.1).withKV(0.12).withKS(0.0);

  // Kicker motor
  public static final double kickerGearRatio = 1.0;
  public static final boolean kickerMotorInverted = false;
  public static final double kickerStatorCurrentLimit = 80.0;
  public static final double kickerFeedVelocityRPM = 4000.0;
  public static final double kickerEjectVelocityRPM = -4000.0;
  public static final Slot0Configs kickerGains =
      new Slot0Configs().withKP(0.1).withKV(0.12).withKS(0.0);

  // Simulation
  public static final double beltSimMOI = 0.001;
  public static final double kickerSimMOI = 0.001;
}
