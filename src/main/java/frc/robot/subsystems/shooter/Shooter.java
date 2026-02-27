package frc.robot.subsystems.shooter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.FieldConstants;
import frc.robot.GameData;
import frc.robot.util.LoggedTunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  // Tunable PID gains
  private static final LoggedTunableNumber shooterKp = new LoggedTunableNumber("Shooter/kP", 0.1);
  private static final LoggedTunableNumber shooterKv = new LoggedTunableNumber("Shooter/kV", 0.12);
  private static final LoggedTunableNumber shooterKs = new LoggedTunableNumber("Shooter/kS", 0.0);

  // Tunable velocity setpoints
  private static final LoggedTunableNumber ejectRPM =
      new LoggedTunableNumber("Shooter/EjectRPM", ShooterConstants.ejectRPM);
  private static final LoggedTunableNumber toleranceRPM =
      new LoggedTunableNumber("Shooter/ToleranceRPM", ShooterConstants.toleranceRPM);

  // RPM override for manual testing (0 = use distance table)
  private static final LoggedTunableNumber rpmOverride =
      new LoggedTunableNumber("Shooter/RPMOverride", 0.0);

  // Hood angle override for servo testing (0 = use distance table, non-zero = use this angle)
  private static final LoggedTunableNumber hoodAngleOverride =
      new LoggedTunableNumber("Shooter/HoodAngleOverride", 0.0);

  // Bang-bang FOC mode (1.0 = enabled, 0.0 = use VelocityVoltage PID)
  private static final LoggedTunableNumber bangBangEnabled =
      new LoggedTunableNumber("Shooter/BangBangEnabled", 0.0);
  private static final LoggedTunableNumber bangBangPeakAmps =
      new LoggedTunableNumber("Shooter/BangBangPeakAmps", ShooterConstants.bangBangPeakCurrentAmps);

  // Enable/disable shoot-on-the-move compensation
  private static final LoggedTunableNumber shootOnMoveEnabled =
      new LoggedTunableNumber("Shooter/ShootOnMoveEnabled", 1.0);

  public enum Goal {
    IDLE,
    SHOOT,
    EJECT
  }

  // IO
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();
  private final ShooterHoodIO hoodIO;
  private final ShooterHoodIOInputsAutoLogged hoodInputs = new ShooterHoodIOInputsAutoLogged();

  // Pose and speed suppliers from drive
  private final Supplier<Pose2d> poseSupplier;
  private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;

  // Interpolation tables
  private final InterpolatingDoubleTreeMap distanceToRPM;
  private final InterpolatingDoubleTreeMap distanceToAngle;
  private final InterpolatingDoubleTreeMap distanceToTOF;

  // State
  private Goal goal = Goal.IDLE;
  private double commandedRPM = 0.0;
  private double commandedAngleDeg = ShooterConstants.hoodMinAngleDeg;
  private double distanceToTarget = 0.0;
  private Translation2d aimTarget = Translation2d.kZero;

  // Alerts
  private final Alert leftDisconnectedAlert =
      new Alert("Shooter left motor disconnected.", AlertType.kError);
  private final Alert rightDisconnectedAlert =
      new Alert("Shooter right motor disconnected.", AlertType.kError);
  private final Alert leftOverTempAlert =
      new Alert("Shooter left motor over temperature.", AlertType.kWarning);
  private final Alert rightOverTempAlert =
      new Alert("Shooter right motor over temperature.", AlertType.kWarning);

  public Shooter(
      ShooterIO io,
      ShooterHoodIO hoodIO,
      Supplier<Pose2d> poseSupplier,
      Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    this.io = io;
    this.hoodIO = hoodIO;
    this.poseSupplier = poseSupplier;
    this.fieldSpeedsSupplier = fieldSpeedsSupplier;
    this.distanceToRPM = ShooterConstants.createDistanceToRPMMap();
    this.distanceToAngle = ShooterConstants.createDistanceToAngleMap();
    this.distanceToTOF = ShooterConstants.createDistanceToTOFMap();
  }

  public void setGoal(Goal goal) {
    this.goal = goal;
  }

  @AutoLogOutput(key = "Shooter/Goal")
  public Goal getGoal() {
    return goal;
  }

  @AutoLogOutput(key = "Shooter/CommandedRPM")
  public double getCommandedRPM() {
    return commandedRPM;
  }

  @AutoLogOutput(key = "Shooter/CommandedAngleDeg")
  public double getCommandedAngleDeg() {
    return commandedAngleDeg;
  }

  @AutoLogOutput(key = "Shooter/DistanceToTarget")
  public double getDistanceToTarget() {
    return distanceToTarget;
  }

  @AutoLogOutput(key = "Shooter/AverageVelocityRPM")
  public double getAverageVelocityRPM() {
    double avgRadPerSec = (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0;
    return radPerSecToRPM(avgRadPerSec);
  }

  /** Returns true if both motors are within tolerance of the commanded setpoint. */
  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public boolean isAtSetpoint() {
    if (goal != Goal.SHOOT || commandedRPM == 0.0) {
      return false;
    }
    double tolerance = rpmToRadPerSec(toleranceRPM.get());
    double setpoint = rpmToRadPerSec(commandedRPM);
    return Math.abs(inputs.leftVelocityRadPerSec - setpoint) < tolerance
        && Math.abs(inputs.rightVelocityRadPerSec - setpoint) < tolerance;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    hoodIO.updateInputs(hoodInputs);
    Logger.processInputs("Shooter/Hood", hoodInputs);

    if (DriverStation.isDisabled()) {
      io.stop();
      hoodIO.stop();
      updateAlerts();
      return;
    }

    // Update gains if tuned
    LoggedTunableNumber.ifChanged(
        hashCode(),
        values -> io.setGains(values[0], values[1], values[2]),
        shooterKp,
        shooterKv,
        shooterKs);
    LoggedTunableNumber.ifChanged(
        hashCode() + 1, values -> io.setPeakTorqueCurrent(values[0]), bangBangPeakAmps);

    switch (goal) {
      case SHOOT:
        commandedRPM = calculateShootRPM();
        commandedAngleDeg = distanceToAngle.get(distanceToTarget);
        if (GameData.canSpinUp(poseSupplier.get().getTranslation())) {
          if (bangBangEnabled.get() > 0.5) {
            // VelocityTorqueCurrentFOC with very high kP acts as bang-bang on the motor controller
            io.setVelocityFOC(rpmToRadPerSec(commandedRPM));
          } else {
            // Standard VelocityVoltage PID
            io.setVelocity(rpmToRadPerSec(commandedRPM));
          }
        } else {
          commandedRPM = 0.0;
          commandedAngleDeg = ShooterConstants.hoodMinAngleDeg;
          io.stop();
        }
        break;
      case EJECT:
        commandedRPM = ejectRPM.get();
        commandedAngleDeg = ShooterConstants.hoodMinAngleDeg;
        io.setVelocity(rpmToRadPerSec(commandedRPM));
        break;
      default:
        commandedRPM = 0.0;
        commandedAngleDeg = ShooterConstants.hoodMinAngleDeg;
        io.stop();
        break;
    }

    // Apply hood angle override for servo testing (non-zero value bypasses distance table)
    if (hoodAngleOverride.get() != 0.0) {
      commandedAngleDeg = hoodAngleOverride.get();
    }
    hoodIO.setAngle(commandedAngleDeg);

    // Log the aim target and hub status for visualization
    Logger.recordOutput(
        "Shooter/AimTarget", new Pose2d(aimTarget, poseSupplier.get().getRotation()));
    Logger.recordOutput("Shooter/HubActive", GameData.isHubActive());

    updateAlerts();
  }

  /** Calculates the target RPM based on distance to target with shoot-on-the-move compensation. */
  private double calculateShootRPM() {
    // Allow manual RPM override for testing
    if (rpmOverride.get() != 0.0) {
      return rpmOverride.get();
    }

    Pose2d robotPose = poseSupplier.get();
    Translation2d robotPosition = robotPose.getTranslation();
    boolean isRedAlliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // Determine real target (hub or passing target)
    Translation2d realTarget;
    if (FieldConstants.isInOwnAllianceZone(robotPosition, isRedAlliance)) {
      realTarget = FieldConstants.getHubCenter(isRedAlliance);
    } else {
      realTarget = FieldConstants.getPassingTarget(robotPosition, isRedAlliance);
    }

    // Apply shoot-on-the-move compensation using virtual target method
    Translation2d compensatedTarget = realTarget;
    if (shootOnMoveEnabled.get() > 0.5) {
      ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
      compensatedTarget =
          computeVirtualTarget(
              robotPosition, realTarget, fieldSpeeds, ShooterConstants.shotCompensationIterations);
    }

    aimTarget = compensatedTarget;
    distanceToTarget = robotPosition.getDistance(compensatedTarget);
    return distanceToRPM.get(distanceToTarget);
  }

  /**
   * Computes the virtual target position to compensate for robot motion. Uses iterative refinement:
   * each iteration recomputes the time-of-flight at the new distance and adjusts the target.
   */
  private Translation2d computeVirtualTarget(
      Translation2d robotPosition,
      Translation2d realTarget,
      ChassisSpeeds fieldSpeeds,
      int iterations) {
    Translation2d virtualTarget = realTarget;
    for (int i = 0; i < iterations; i++) {
      double distance = robotPosition.getDistance(virtualTarget);
      double tof = distanceToTOF.get(distance);
      virtualTarget =
          new Translation2d(
              realTarget.getX() - fieldSpeeds.vxMetersPerSecond * tof,
              realTarget.getY() - fieldSpeeds.vyMetersPerSecond * tof);
    }
    return virtualTarget;
  }

  private void updateAlerts() {
    leftDisconnectedAlert.set(!inputs.leftConnected && Constants.currentMode != Mode.SIM);
    rightDisconnectedAlert.set(!inputs.rightConnected && Constants.currentMode != Mode.SIM);
    leftOverTempAlert.set(inputs.leftTempCelsius > 80.0);
    rightOverTempAlert.set(inputs.rightTempCelsius > 80.0);
  }

  private static double rpmToRadPerSec(double rpm) {
    return rpm * 2.0 * Math.PI / 60.0;
  }

  private static double radPerSecToRPM(double radPerSec) {
    return radPerSec * 60.0 / (2.0 * Math.PI);
  }

  // Command factories
  public Command setGoalCommand(Goal goal) {
    return Commands.runOnce(() -> setGoal(goal)).withName("Shooter " + goal.name());
  }

  public Command shootCommand() {
    return Commands.startEnd(() -> setGoal(Goal.SHOOT), () -> setGoal(Goal.IDLE), this)
        .withName("Shooter Shoot");
  }

  public Command ejectCommand() {
    return Commands.startEnd(() -> setGoal(Goal.EJECT), () -> setGoal(Goal.IDLE), this)
        .withName("Shooter Eject");
  }

  /** Run shooter motors at a raw voltage for characterization. Bypasses goal logic. */
  public void runCharacterization(double volts) {
    io.setVoltage(volts);
  }

  /** Returns the average velocity of both shooter motors in rad/s for characterization. */
  public double getCharacterizationVelocity() {
    return (inputs.leftVelocityRadPerSec + inputs.rightVelocityRadPerSec) / 2.0;
  }

  /**
   * Ramps voltage on the shooter motors and collects velocity/voltage samples, then calculates kS
   * and kV via linear regression. Run from the auto chooser, cancel to see results.
   */
  public static Command shooterFFCharacterization(Shooter shooter) {
    double rampRate = 0.25; // Volts per second (slower for high-inertia flywheel)
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return Commands.sequence(
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
                  shooter.runCharacterization(voltage);
                  velocitySamples.add(shooter.getCharacterizationVelocity());
                  voltageSamples.add(voltage);
                },
                shooter)
            .finallyDo(
                () -> {
                  shooter.runCharacterization(0.0);

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
                  System.out.println("********** Shooter FF Characterization Results **********");
                  System.out.println("\tkS: " + formatter.format(kS));
                  System.out.println("\tkV: " + formatter.format(kV));
                }));
  }
}
