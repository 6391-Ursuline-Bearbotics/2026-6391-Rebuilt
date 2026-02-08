package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class IntakeRollerIOSim implements IntakeRollerIO {
  private static final DCMotor GEARBOX = DCMotor.getFalcon500(1);
  private static final double SIM_KP = 0.05;
  private static final double SIM_KV = 0.12;
  private static final double SIM_KS = 0.0;

  private final DCMotorSim sim;
  private final PIDController controller = new PIDController(SIM_KP, 0, 0);

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;

  public IntakeRollerIOSim() {
    sim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                GEARBOX, IntakeConstants.rollerSimMOI, IntakeConstants.rollerGearRatio),
            GEARBOX);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    if (closedLoop) {
      appliedVolts = ffVolts + controller.calculate(sim.getAngularVelocityRadPerSec());
    } else {
      controller.reset();
    }

    sim.setInputVoltage(MathUtil.clamp(appliedVolts, -12.0, 12.0));
    sim.update(0.02);

    inputs.connected = true;
    inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = appliedVolts;
    inputs.statorCurrentAmps = Math.abs(sim.getCurrentDrawAmps());
    inputs.supplyCurrentAmps = inputs.statorCurrentAmps * Math.abs(appliedVolts) / 12.0;
    inputs.tempCelsius = 25.0;
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    closedLoop = true;
    ffVolts = SIM_KS * Math.signum(velocityRadPerSec) + SIM_KV * velocityRadPerSec;
    controller.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
  }
}
