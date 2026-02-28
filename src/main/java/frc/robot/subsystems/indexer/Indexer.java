package frc.robot.subsystems.indexer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.GameData;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.Supplier;
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

  // Tunable jam detection parameters
  private static final LoggedTunableNumber jamCurrentThreshold =
      new LoggedTunableNumber("Indexer/JamCurrentThreshold", IndexerConstants.jamCurrentThreshold);
  private static final LoggedTunableNumber jamDebounceTime =
      new LoggedTunableNumber("Indexer/JamDebounceTime", IndexerConstants.jamDebounceTime);
  private static final LoggedTunableNumber jamReverseTime =
      new LoggedTunableNumber("Indexer/JamReverseTime", IndexerConstants.jamReverseTime);

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

  // Pose supplier for game data gating
  private final Supplier<Pose2d> poseSupplier;

  // State
  private Goal goal = Goal.IDLE;

  // Jam detection
  private final Timer jamTimer = new Timer();
  private final Timer jamReverseTimer = new Timer();
  private boolean jammed = false;
  private Goal goalBeforeJam = Goal.IDLE;

  // Alerts
  private final Alert beltDisconnectedAlert =
      new Alert("Indexer belt motor disconnected.", AlertType.kError);
  private final Alert kickerDisconnectedAlert =
      new Alert("Indexer kicker motor disconnected.", AlertType.kError);
  private final Alert beltOverTempAlert =
      new Alert("Indexer belt motor over temperature.", AlertType.kWarning);
  private final Alert kickerOverTempAlert =
      new Alert("Indexer kicker motor over temperature.", AlertType.kWarning);
  private final Alert jamAlert = new Alert("Indexer jam detected.", AlertType.kWarning);

  public Indexer(IndexerBeltIO beltIO, IndexerKickerIO kickerIO, Supplier<Pose2d> poseSupplier) {
    this.beltIO = beltIO;
    this.kickerIO = kickerIO;
    this.poseSupplier = poseSupplier;
    jamTimer.start();
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

    // Determine commanded velocities and whether motors are active
    double commandedBeltVel = 0.0;
    double commandedKickerVel = 0.0;
    boolean motorsActive = false;

    switch (goal) {
      case FEED:
        if (GameData.canFeed(poseSupplier.get().getTranslation())) {
          commandedBeltVel = rpmToRadPerSec(beltFeedRPM.get());
          commandedKickerVel = rpmToRadPerSec(kickerFeedRPM.get());
          motorsActive = true;
        }
        break;
      case EJECT:
        commandedBeltVel = rpmToRadPerSec(beltEjectRPM.get());
        commandedKickerVel = rpmToRadPerSec(kickerEjectRPM.get());
        motorsActive = true;
        break;
      default:
        break;
    }

    // Jam detection: either belt or kicker stalled while motors are active
    boolean beltStall =
        motorsActive
            && beltInputs.statorCurrentAmps > jamCurrentThreshold.get()
            && Math.abs(beltInputs.velocityRadPerSec) < Math.abs(commandedBeltVel) * 0.1;
    boolean kickerStall =
        motorsActive
            && kickerInputs.statorCurrentAmps > jamCurrentThreshold.get()
            && Math.abs(kickerInputs.velocityRadPerSec) < Math.abs(commandedKickerVel) * 0.1;

    if (!beltStall && !kickerStall) {
      jamTimer.restart();
    }

    // Handle jam state
    if (jammed) {
      // Reversing both motors to clear jam
      beltIO.setVelocity(rpmToRadPerSec(beltEjectRPM.get()));
      kickerIO.setVelocity(rpmToRadPerSec(kickerEjectRPM.get()));
      if (jamReverseTimer.hasElapsed(jamReverseTime.get())) {
        // Done reversing, resume previous goal
        jammed = false;
        goal = goalBeforeJam;
      }
    } else if (motorsActive && jamTimer.hasElapsed(jamDebounceTime.get())) {
      // Jam detected — start reversing both motors
      jammed = true;
      goalBeforeJam = goal;
      jamReverseTimer.restart();
      beltIO.setVelocity(rpmToRadPerSec(beltEjectRPM.get()));
      kickerIO.setVelocity(rpmToRadPerSec(kickerEjectRPM.get()));
    } else if (motorsActive) {
      beltIO.setVelocity(commandedBeltVel);
      kickerIO.setVelocity(commandedKickerVel);
    } else {
      beltIO.stop();
      kickerIO.stop();
      jammed = false;
    }

    updateAlerts();
  }

  @AutoLogOutput(key = "Indexer/Jammed")
  public boolean isJammed() {
    return jammed;
  }

  private void updateAlerts() {
    beltDisconnectedAlert.set(!beltInputs.connected && Constants.currentMode != Mode.SIM);
    kickerDisconnectedAlert.set(!kickerInputs.connected && Constants.currentMode != Mode.SIM);
    beltOverTempAlert.set(beltInputs.tempCelsius > 80.0);
    kickerOverTempAlert.set(kickerInputs.tempCelsius > 80.0);
    jamAlert.set(jammed);
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
