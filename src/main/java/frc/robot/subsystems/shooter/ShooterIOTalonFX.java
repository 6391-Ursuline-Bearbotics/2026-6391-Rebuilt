package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX leftTalon;
  private final TalonFX rightTalon;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0.0);

  // Left motor signals
  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftStatorCurrent;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Temperature> leftTemp;

  // Right motor signals
  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightStatorCurrent;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Temperature> rightTemp;

  private final Debouncer leftConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer rightConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ShooterIOTalonFX() {
    leftTalon = new TalonFX(ShooterConstants.leftMotorId, ShooterConstants.canBus);
    rightTalon = new TalonFX(ShooterConstants.rightMotorId, ShooterConstants.canBus);

    // Configure left motor
    var leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    leftConfig.MotorOutput.Inverted =
        ShooterConstants.leftMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    leftConfig.Slot0 = ShooterConstants.gains;
    leftConfig.Feedback.SensorToMechanismRatio = ShooterConstants.gearRatio;
    leftConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.statorCurrentLimit;
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> leftTalon.getConfigurator().apply(leftConfig, 0.25));

    // Configure right motor
    var rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    rightConfig.MotorOutput.Inverted =
        ShooterConstants.rightMotorInverted
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;
    rightConfig.Slot0 = ShooterConstants.gains;
    rightConfig.Feedback.SensorToMechanismRatio = ShooterConstants.gearRatio;
    rightConfig.CurrentLimits.StatorCurrentLimit = ShooterConstants.statorCurrentLimit;
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    tryUntilOk(5, () -> rightTalon.getConfigurator().apply(rightConfig, 0.25));

    // Get status signals
    leftVelocity = leftTalon.getVelocity();
    leftAppliedVolts = leftTalon.getMotorVoltage();
    leftStatorCurrent = leftTalon.getStatorCurrent();
    leftSupplyCurrent = leftTalon.getSupplyCurrent();
    leftTemp = leftTalon.getDeviceTemp();

    rightVelocity = rightTalon.getVelocity();
    rightAppliedVolts = rightTalon.getMotorVoltage();
    rightStatorCurrent = rightTalon.getStatorCurrent();
    rightSupplyCurrent = rightTalon.getSupplyCurrent();
    rightTemp = rightTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftVelocity,
        leftAppliedVolts,
        leftStatorCurrent,
        leftSupplyCurrent,
        leftTemp,
        rightVelocity,
        rightAppliedVolts,
        rightStatorCurrent,
        rightSupplyCurrent,
        rightTemp);
    ParentDevice.optimizeBusUtilizationForAll(leftTalon, rightTalon);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var leftStatus =
        BaseStatusSignal.refreshAll(
            leftVelocity, leftAppliedVolts, leftStatorCurrent, leftSupplyCurrent, leftTemp);
    var rightStatus =
        BaseStatusSignal.refreshAll(
            rightVelocity, rightAppliedVolts, rightStatorCurrent, rightSupplyCurrent, rightTemp);

    inputs.leftConnected = leftConnectedDebounce.calculate(leftStatus.isOK());
    inputs.leftVelocityRadPerSec = Units.rotationsToRadians(leftVelocity.getValueAsDouble());
    inputs.leftAppliedVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftStatorCurrentAmps = leftStatorCurrent.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
    inputs.leftTempCelsius = leftTemp.getValueAsDouble();

    inputs.rightConnected = rightConnectedDebounce.calculate(rightStatus.isOK());
    inputs.rightVelocityRadPerSec = Units.rotationsToRadians(rightVelocity.getValueAsDouble());
    inputs.rightAppliedVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightStatorCurrentAmps = rightStatorCurrent.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
    inputs.rightTempCelsius = rightTemp.getValueAsDouble();
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    double velocityRotPerSec = Units.radiansToRotations(velocityRadPerSec);
    leftTalon.setControl(velocityRequest.withVelocity(velocityRotPerSec));
    rightTalon.setControl(velocityRequest.withVelocity(velocityRotPerSec));
  }

  @Override
  public void setVoltage(double volts) {
    leftTalon.setControl(voltageRequest.withOutput(volts));
    rightTalon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    leftTalon.setControl(voltageRequest.withOutput(0.0));
    rightTalon.setControl(voltageRequest.withOutput(0.0));
  }

  @Override
  public void setGains(double kP, double kV, double kS) {
    var gains = new Slot0Configs().withKP(kP).withKV(kV).withKS(kS);
    tryUntilOk(5, () -> leftTalon.getConfigurator().apply(gains, 0.25));
    tryUntilOk(5, () -> rightTalon.getConfigurator().apply(gains, 0.25));
  }
}
