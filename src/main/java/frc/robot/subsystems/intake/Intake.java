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
import frc.robot.util.LoggedTunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  // Tunable roller gains
  private static final LoggedTunableNumber rollerKp =
      new LoggedTunableNumber("Intake/Roller/kP", 0.6);
  private static final LoggedTunableNumber rollerKv =
      new LoggedTunableNumber("Intake/Roller/kV", 0.119);
  private static final LoggedTunableNumber rollerKs =
      new LoggedTunableNumber("Intake/Roller/kS", 0.24691);

  // Tunable roller velocity setpoints
  private static final LoggedTunableNumber rollerIntakeRPM =
      new LoggedTunableNumber("Intake/Roller/IntakeRPM", IntakeConstants.rollerIntakeVelocityRPM);
  private static final LoggedTunableNumber rollerClumpIntakeRPM =
      new LoggedTunableNumber(
          "Intake/Roller/ClumpIntakeRPM", IntakeConstants.rollerClumpIntakeVelocityRPM);
  private static final LoggedTunableNumber rollerEjectRPM =
      new LoggedTunableNumber("Intake/Roller/EjectRPM", IntakeConstants.rollerEjectVelocityRPM);

  // Tunable deploy parameters
  private static final LoggedTunableNumber deployVoltage =
      new LoggedTunableNumber("Intake/Deploy/Voltage", IntakeConstants.deployVoltage);
  private static final LoggedTunableNumber retractVoltage =
      new LoggedTunableNumber("Intake/Deploy/RetractVoltage", IntakeConstants.retractVoltage);
  private static final LoggedTunableNumber stallCurrentThreshold =
      new LoggedTunableNumber(
          "Intake/Deploy/StallCurrentThreshold", IntakeConstants.deployCurrentThreshold);

  // Tunable roller jam detection parameters
  private static final LoggedTunableNumber rollerJamCurrentThreshold =
      new LoggedTunableNumber(
          "Intake/Roller/JamCurrentThreshold", IntakeConstants.rollerJamCurrentThreshold);
  private static final LoggedTunableNumber rollerJamDebounceTime =
      new LoggedTunableNumber(
          "Intake/Roller/JamDebounceTime", IntakeConstants.rollerJamDebounceTime);
  private static final LoggedTunableNumber rollerJamReverseTime =
      new LoggedTunableNumber("Intake/Roller/JamReverseTime", IntakeConstants.rollerJamReverseTime);

  /** Desired intake behavior, set by operator controls. */
  public enum Goal {
    IDLE, // Retracted, rollers off
    INTAKE, // Deployed, rollers spinning inward
    EJECT, // Deployed, rollers spinning outward
    DEPLOYED_IDLE, // Deployed, rollers off
    CLUMP_INTAKE // Deployed, rollers at higher speed for clumps
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

  // Roller jam detection
  private final Timer rollerJamTimer = new Timer();
  private final Timer rollerJamReverseTimer = new Timer();
  private boolean rollerJammed = false;
  private Goal goalBeforeJam = Goal.IDLE;

  // Alerts
  private final Alert deployDisconnectedAlert =
      new Alert("Intake deploy motor disconnected.", AlertType.kError);
  private final Alert rollerDisconnectedAlert =
      new Alert("Intake roller motor disconnected.", AlertType.kError);
  private final Alert deployOverTempAlert =
      new Alert("Intake deploy motor over temperature.", AlertType.kWarning);
  private final Alert rollerOverTempAlert =
      new Alert("Intake roller motor over temperature.", AlertType.kWarning);
  private final Alert rollerJamAlert = new Alert("Intake roller jam detected.", AlertType.kWarning);

  public Intake(IntakeDeployIO deployIO, IntakeRollerIO rollerIO) {
    this.deployIO = deployIO;
    this.rollerIO = rollerIO;
    stallTimer.start();
    rollerJamTimer.start();
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

  @AutoLogOutput(key = "Intake/RollerVelocityRPM")
  public double getRollerVelocityRPM() {
    return rollerInputs.velocityRadPerSec * 60.0 / (2.0 * Math.PI);
  }

  /** Run roller at raw voltage for characterization. */
  public void runRollerCharacterization(double volts) {
    rollerIO.setVoltage(volts);
  }

  /** Get roller velocity in rad/s for characterization. */
  public double getRollerCharacterizationVelocity() {
    return rollerInputs.velocityRadPerSec;
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
        goal == Goal.INTAKE
            || goal == Goal.EJECT
            || goal == Goal.DEPLOYED_IDLE
            || goal == Goal.CLUMP_INTAKE;

    // Current spike detection: ignore inrush, then check threshold
    boolean stallDetected =
        stallTimer.hasElapsed(IntakeConstants.deployInrushIgnoreTime)
            && deployInputs.statorCurrentAmps > stallCurrentThreshold.get();

    // Deploy state machine
    switch (deployState) {
      case RETRACTED:
        if (goalWantsDeploy) {
          transitionTo(DeployState.DEPLOYING);
          deployIO.setVoltage(deployVoltage.get());
        } else {
          deployIO.stop();
        }
        break;

      case DEPLOYING:
        if (!goalWantsDeploy) {
          transitionTo(DeployState.RETRACTING);
          deployIO.setVoltage(retractVoltage.get());
        } else if (stallDetected) {
          transitionTo(DeployState.DEPLOYED);
          deployIO.stop();
        } else {
          deployIO.setVoltage(deployVoltage.get());
        }
        break;

      case DEPLOYED:
        if (!goalWantsDeploy) {
          transitionTo(DeployState.RETRACTING);
          deployIO.setVoltage(retractVoltage.get());
        } else {
          deployIO.stop();
        }
        break;

      case RETRACTING:
        if (goalWantsDeploy) {
          transitionTo(DeployState.DEPLOYING);
          deployIO.setVoltage(deployVoltage.get());
        } else if (stallDetected) {
          transitionTo(DeployState.RETRACTED);
          deployIO.stop();
        } else {
          deployIO.setVoltage(retractVoltage.get());
        }
        break;
    }

    // Update roller gains if tuned
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> rollerIO.setGains(values[0], values[1], values[2]),
        rollerKp,
        rollerKv,
        rollerKs);

    // Roller control: only run when fully deployed
    if (deployState == DeployState.DEPLOYED) {
      // Determine commanded velocity for current goal
      double commandedVelRadPerSec = 0.0;
      boolean rollerActive = false;
      switch (goal) {
        case INTAKE:
          commandedVelRadPerSec = rpmToRadPerSec(rollerIntakeRPM.get());
          rollerActive = true;
          break;
        case CLUMP_INTAKE:
          commandedVelRadPerSec = rpmToRadPerSec(rollerClumpIntakeRPM.get());
          rollerActive = true;
          break;
        case EJECT:
          commandedVelRadPerSec = rpmToRadPerSec(rollerEjectRPM.get());
          rollerActive = true;
          break;
        default:
          break;
      }

      // Jam detection: high current + near-zero velocity while motor is commanded
      boolean stallCondition =
          rollerActive
              && rollerInputs.statorCurrentAmps > rollerJamCurrentThreshold.get()
              && Math.abs(rollerInputs.velocityRadPerSec) < Math.abs(commandedVelRadPerSec) * 0.1;

      if (!stallCondition) {
        rollerJamTimer.restart();
      }

      // Handle jam state
      if (rollerJammed) {
        // Reversing to clear jam
        rollerIO.setVelocity(rpmToRadPerSec(rollerEjectRPM.get()));
        if (rollerJamReverseTimer.hasElapsed(rollerJamReverseTime.get())) {
          // Done reversing, resume previous goal
          rollerJammed = false;
          goal = goalBeforeJam;
        }
      } else if (rollerActive && rollerJamTimer.hasElapsed(rollerJamDebounceTime.get())) {
        // Jam detected — start reversing
        rollerJammed = true;
        goalBeforeJam = goal;
        rollerJamReverseTimer.restart();
        rollerIO.setVelocity(rpmToRadPerSec(rollerEjectRPM.get()));
      } else if (rollerActive) {
        rollerIO.setVelocity(commandedVelRadPerSec);
      } else {
        rollerIO.stop();
        rollerJammed = false;
      }
    } else {
      rollerIO.stop();
      rollerJammed = false;
    }

    updateAlerts();
  }

  private void transitionTo(DeployState newState) {
    // Switch to coast when deployed so intake can be pushed back
    if (newState == DeployState.DEPLOYED) {
      deployIO.setBrakeMode(false);
    } else if (deployState == DeployState.DEPLOYED) {
      deployIO.setBrakeMode(true);
    }
    deployState = newState;
    stallTimer.restart();
  }

  @AutoLogOutput(key = "Intake/Roller/Jammed")
  public boolean isRollerJammed() {
    return rollerJammed;
  }

  private void updateAlerts() {
    deployDisconnectedAlert.set(!deployInputs.connected && Constants.currentMode != Mode.SIM);
    rollerDisconnectedAlert.set(!rollerInputs.connected && Constants.currentMode != Mode.SIM);
    deployOverTempAlert.set(deployInputs.tempCelsius > 80.0);
    rollerOverTempAlert.set(rollerInputs.tempCelsius > 80.0);
    rollerJamAlert.set(rollerJammed);
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

  /**
   * Ramps voltage on the roller motor and collects velocity/voltage samples, then calculates kS and
   * kV via linear regression. Run from the auto chooser, cancel to see results.
   */
  public static Command rollerFFCharacterization(Intake intake) {
    double rampRate = 0.5; // Volts per second
    double startDelay = 1.0; // Seconds to let deploy settle
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
        // Deploy the intake first and wait for it to settle
        Commands.runOnce(() -> intake.setGoal(Goal.DEPLOYED_IDLE)),
        Commands.waitSeconds(startDelay),

        // Reset data
        Commands.runOnce(
            () -> {
              velocitySamples.clear();
              voltageSamples.clear();
            }),

        // Start timer
        Commands.runOnce(timer::restart),

        // Ramp voltage and collect samples
        Commands.run(
                () -> {
                  double voltage = timer.get() * rampRate;
                  intake.runRollerCharacterization(voltage);
                  velocitySamples.add(intake.getRollerCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                intake)
            .finallyDo(
                () -> {
                  intake.runRollerCharacterization(0.0);
                  intake.setGoal(Goal.IDLE);

                  int n = velocitySamples.size();
                  double sumX = 0.0;
                  double sumY = 0.0;
                  double sumXY = 0.0;
                  double sumX2 = 0.0;
                  for (int i = 0; i < n; i++) {
                    sumX += velocitySamples.get(i);
                    sumY += voltageSamples.get(i);
                    sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                    sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                  }
                  double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                  double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                  NumberFormat formatter = new DecimalFormat("#0.00000");
                  System.out.println(
                      "********** Intake Roller FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }
}
