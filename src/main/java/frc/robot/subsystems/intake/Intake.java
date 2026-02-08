package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Desired intake behavior, set by operator controls. */
  public enum Goal {
    IDLE, // Retracted, rollers off
    INTAKE, // Deployed, rollers spinning inward
    EJECT, // Deployed, rollers spinning outward
    DEPLOYED_IDLE // Deployed, rollers off
  }

  /** Internal deploy mechanism state. */
  private enum DeployState {
    RETRACTED,
    DEPLOYING,
    DEPLOYED,
    RETRACTING
  }

  // IO layers
  private final IntakeDeployIO deployIO;
  private final IntakeDeployIOInputsAutoLogged deployInputs = new IntakeDeployIOInputsAutoLogged();
  private final IntakeRollerIO rollerIO;
  private final IntakeRollerIOInputsAutoLogged rollerInputs = new IntakeRollerIOInputsAutoLogged();

  // State
  private Goal goal = Goal.IDLE;
  private DeployState deployState = DeployState.RETRACTED;

  // Deploy current spike detection
  private final Timer stallTimer = new Timer();

  // Alerts
  private final Alert deployDisconnectedAlert =
      new Alert("Intake deploy motor disconnected.", AlertType.kError);
  private final Alert rollerDisconnectedAlert =
      new Alert("Intake roller motor disconnected.", AlertType.kError);
  private final Alert deployOverTempAlert =
      new Alert("Intake deploy motor over temperature.", AlertType.kWarning);
  private final Alert rollerOverTempAlert =
      new Alert("Intake roller motor over temperature.", AlertType.kWarning);

  public Intake(IntakeDeployIO deployIO, IntakeRollerIO rollerIO) {
    this.deployIO = deployIO;
    this.rollerIO = rollerIO;
    stallTimer.start();
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  @AutoLogOutput(key = "Intake/Goal")
  public Goal getGoal() {
    return goal;
  }

  @AutoLogOutput(key = "Intake/DeployState")
  public DeployState getDeployState() {
    return deployState;
  }

  @AutoLogOutput(key = "Intake/Deployed")
  public boolean isDeployed() {
    return deployState == DeployState.DEPLOYED;
  }

  @AutoLogOutput(key = "Intake/Retracted")
  public boolean isRetracted() {
    return deployState == DeployState.RETRACTED;
  }

  public double getRollerVelocityRPM() {
    return rollerInputs.velocityRadPerSec * 60.0 / (2.0 * Math.PI);
  }

  @Override
  public void periodic() {
    // Update and log inputs
    deployIO.updateInputs(deployInputs);
    Logger.processInputs("Intake/Deploy", deployInputs);
    rollerIO.updateInputs(rollerInputs);
    Logger.processInputs("Intake/Roller", rollerInputs);

    // Stop everything when disabled
    if (DriverStation.isDisabled()) {
      deployIO.stop();
      rollerIO.stop();
      updateAlerts();
      return;
    }

    // Determine if goal wants the intake deployed
    boolean goalWantsDeploy =
        goal == Goal.INTAKE || goal == Goal.EJECT || goal == Goal.DEPLOYED_IDLE;

    // Current spike detection: ignore inrush, then check threshold
    boolean stallDetected =
        stallTimer.hasElapsed(IntakeConstants.deployInrushIgnoreTime)
            && deployInputs.statorCurrentAmps > IntakeConstants.deployCurrentThreshold;

    // Deploy state machine
    switch (deployState) {
      case RETRACTED:
        if (goalWantsDeploy) {
          transitionTo(DeployState.DEPLOYING);
          deployIO.setVoltage(IntakeConstants.deployVoltage);
        } else {
          deployIO.stop();
        }
        break;

      case DEPLOYING:
        if (!goalWantsDeploy) {
          transitionTo(DeployState.RETRACTING);
          deployIO.setVoltage(IntakeConstants.retractVoltage);
        } else if (stallDetected) {
          transitionTo(DeployState.DEPLOYED);
          deployIO.setVoltage(IntakeConstants.deployHoldVoltage);
        } else {
          deployIO.setVoltage(IntakeConstants.deployVoltage);
        }
        break;

      case DEPLOYED:
        if (!goalWantsDeploy) {
          transitionTo(DeployState.RETRACTING);
          deployIO.setVoltage(IntakeConstants.retractVoltage);
        } else {
          deployIO.setVoltage(IntakeConstants.deployHoldVoltage);
        }
        break;

      case RETRACTING:
        if (goalWantsDeploy) {
          transitionTo(DeployState.DEPLOYING);
          deployIO.setVoltage(IntakeConstants.deployVoltage);
        } else if (stallDetected) {
          transitionTo(DeployState.RETRACTED);
          deployIO.stop();
        } else {
          deployIO.setVoltage(IntakeConstants.retractVoltage);
        }
        break;
    }

    // Roller control: only run when fully deployed
    if (deployState == DeployState.DEPLOYED) {
      switch (goal) {
        case INTAKE:
          rollerIO.setVelocity(rpmToRadPerSec(IntakeConstants.rollerIntakeVelocityRPM));
          break;
        case EJECT:
          rollerIO.setVelocity(rpmToRadPerSec(IntakeConstants.rollerEjectVelocityRPM));
          break;
        default:
          rollerIO.stop();
          break;
      }
    } else {
      rollerIO.stop();
    }

    updateAlerts();
  }

  private void transitionTo(DeployState newState) {
    deployState = newState;
    stallTimer.restart();
  }

  private void updateAlerts() {
    deployDisconnectedAlert.set(!deployInputs.connected && Constants.currentMode != Mode.SIM);
    rollerDisconnectedAlert.set(!rollerInputs.connected && Constants.currentMode != Mode.SIM);
    deployOverTempAlert.set(deployInputs.tempCelsius > 80.0);
    rollerOverTempAlert.set(rollerInputs.tempCelsius > 80.0);
  }

  private static double rpmToRadPerSec(double rpm) {
    return rpm * 2.0 * Math.PI / 60.0;
  }

  // Command factories
  public Command setGoalCommand(Goal goal) {
    return Commands.runOnce(() -> setGoal(goal)).withName("Intake " + goal.name());
  }

  public Command intakeCommand() {
    return Commands.startEnd(() -> setGoal(Goal.INTAKE), () -> setGoal(Goal.IDLE), this)
        .withName("Intake");
  }

  public Command ejectCommand() {
    return Commands.startEnd(() -> setGoal(Goal.EJECT), () -> setGoal(Goal.IDLE), this)
        .withName("Eject");
  }
}
