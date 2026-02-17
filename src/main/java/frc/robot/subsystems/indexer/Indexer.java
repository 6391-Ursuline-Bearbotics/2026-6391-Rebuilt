package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Indexer extends SubsystemBase {
  // Tunable belt gains
  private static final LoggedTunableNumber beltKp = new LoggedTunableNumber("Indexer/Belt/kP", 0.1);
  private static final LoggedTunableNumber beltKv =
      new LoggedTunableNumber("Indexer/Belt/kV", 0.12);
  private static final LoggedTunableNumber beltKs = new LoggedTunableNumber("Indexer/Belt/kS", 0.0);

  // Tunable kicker gains
  private static final LoggedTunableNumber kickerKp =
      new LoggedTunableNumber("Indexer/Kicker/kP", 0.1);
  private static final LoggedTunableNumber kickerKv =
      new LoggedTunableNumber("Indexer/Kicker/kV", 0.12);
  private static final LoggedTunableNumber kickerKs =
      new LoggedTunableNumber("Indexer/Kicker/kS", 0.0);

  // Tunable velocity setpoints
  private static final LoggedTunableNumber beltFeedRPM =
      new LoggedTunableNumber("Indexer/Belt/FeedRPM", IndexerConstants.beltFeedVelocityRPM);
  private static final LoggedTunableNumber beltEjectRPM =
      new LoggedTunableNumber("Indexer/Belt/EjectRPM", IndexerConstants.beltEjectVelocityRPM);
  private static final LoggedTunableNumber kickerFeedRPM =
      new LoggedTunableNumber("Indexer/Kicker/FeedRPM", IndexerConstants.kickerFeedVelocityRPM);
  private static final LoggedTunableNumber kickerEjectRPM =
      new LoggedTunableNumber("Indexer/Kicker/EjectRPM", IndexerConstants.kickerEjectVelocityRPM);

  public enum Goal {
    IDLE,
    FEED,
    EJECT
  }

  // IO layers
  private final IndexerBeltIO beltIO;
  private final IndexerBeltIOInputsAutoLogged beltInputs = new IndexerBeltIOInputsAutoLogged();
  private final IndexerKickerIO kickerIO;
  private final IndexerKickerIOInputsAutoLogged kickerInputs =
      new IndexerKickerIOInputsAutoLogged();

  // State
  private Goal goal = Goal.IDLE;

  // Alerts
  private final Alert beltDisconnectedAlert =
      new Alert("Indexer belt motor disconnected.", AlertType.kError);
  private final Alert kickerDisconnectedAlert =
      new Alert("Indexer kicker motor disconnected.", AlertType.kError);
  private final Alert beltOverTempAlert =
      new Alert("Indexer belt motor over temperature.", AlertType.kWarning);
  private final Alert kickerOverTempAlert =
      new Alert("Indexer kicker motor over temperature.", AlertType.kWarning);

  public Indexer(IndexerBeltIO beltIO, IndexerKickerIO kickerIO) {
    this.beltIO = beltIO;
    this.kickerIO = kickerIO;
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  @AutoLogOutput(key = "Indexer/Goal")
  public Goal getGoal() {
    return goal;
  }

  @Override
  public void periodic() {
    // Update and log inputs
    beltIO.updateInputs(beltInputs);
    Logger.processInputs("Indexer/Belt", beltInputs);
    kickerIO.updateInputs(kickerInputs);
    Logger.processInputs("Indexer/Kicker", kickerInputs);

    // Stop everything when disabled
    if (DriverStation.isDisabled()) {
      beltIO.stop();
      kickerIO.stop();
      updateAlerts();
      return;
    }

    // Update belt gains if tuned
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> beltIO.setGains(values[0], values[1], values[2]),
        beltKp,
        beltKv,
        beltKs);

    // Update kicker gains if tuned
    LoggedTunableNumber.ifChanged(
        hashCode() + 1,
        values -> kickerIO.setGains(values[0], values[1], values[2]),
        kickerKp,
        kickerKv,
        kickerKs);

    // Motor control based on goal
    switch (goal) {
      case FEED:
        beltIO.setVelocity(rpmToRadPerSec(beltFeedRPM.get()));
        kickerIO.setVelocity(rpmToRadPerSec(kickerFeedRPM.get()));
        break;
      case EJECT:
        beltIO.setVelocity(rpmToRadPerSec(beltEjectRPM.get()));
        kickerIO.setVelocity(rpmToRadPerSec(kickerEjectRPM.get()));
        break;
      default:
        beltIO.stop();
        kickerIO.stop();
        break;
    }

    updateAlerts();
  }

  private void updateAlerts() {
    beltDisconnectedAlert.set(!beltInputs.connected && Constants.currentMode != Mode.SIM);
    kickerDisconnectedAlert.set(!kickerInputs.connected && Constants.currentMode != Mode.SIM);
    beltOverTempAlert.set(beltInputs.tempCelsius > 80.0);
    kickerOverTempAlert.set(kickerInputs.tempCelsius > 80.0);
  }

  private static double rpmToRadPerSec(double rpm) {
    return rpm * 2.0 * Math.PI / 60.0;
  }

  // Command factories
  public Command setGoalCommand(Goal goal) {
    return Commands.runOnce(() -> setGoal(goal)).withName("Indexer " + goal.name());
  }

  public Command feedCommand() {
    return Commands.startEnd(() -> setGoal(Goal.FEED), () -> setGoal(Goal.IDLE), this)
        .withName("Indexer Feed");
  }

  public Command ejectCommand() {
    return Commands.startEnd(() -> setGoal(Goal.EJECT), () -> setGoal(Goal.IDLE), this)
        .withName("Indexer Eject");
  }
}
