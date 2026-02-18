package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;

public final class IntakeConstants {
  // CAN IDs (placeholder - update to match robot wiring)
  public static final int deployMotorId = 4;
  public static final int rollerMotorId = 3;
  public static final CANBus canBus = new CANBus("");

  // Deploy motor
  public static final double deployGearRatio = 25.0;
  public static final boolean deployMotorInverted = false;
  public static final double deployVoltage = 1.0;
  public static final double retractVoltage = -1.0;
  public static final double deployCurrentThreshold = 25.0; // Stator amps indicating hard stop
  public static final double deployInrushIgnoreTime = 0.15; // Seconds to ignore after motor starts
  public static final double deployStatorCurrentLimit = 60.0;

  // Roller motor
  public static final double rollerGearRatio = 1.0;
  public static final boolean rollerMotorInverted = false;
  public static final double rollerIntakeVelocityRPM = 3500.0;
  public static final double rollerEjectVelocityRPM = -3500.0;
  public static final double rollerStatorCurrentLimit = 80.0;
  public static final Slot0Configs rollerGains =
      new Slot0Configs().withKP(0.1).withKV(0.12).withKS(0.0);

  // Simulation
  public static final double deploySimMOI = 0.01;
  public static final double rollerSimMOI = 0.001;
}
