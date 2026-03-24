package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import java.util.Optional;

public class AutoRoutines {
  private final AutoFactory factory;
  private final Drive drive;
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;

  public AutoRoutines(
      AutoFactory factory, Drive drive, Intake intake, Indexer indexer, Shooter shooter) {
    this.factory = factory;
    this.drive = drive;
    this.intake = intake;
    this.indexer = indexer;
    this.shooter = shooter;
  }

  public AutoRoutine resetOdometryStart() {
    AutoRoutine routine = factory.newRoutine("ResetOdometryStart");

    routine
        .active()
        .onTrue(
            factory
                .resetOdometry(() -> Optional.of(new Pose2d(3.559, 4.0296, Rotation2d.kZero)))
                .andThen(Commands.print("Odometry reset to start position")));

    return routine;
  }

  public AutoRoutine depotCycle() {
    AutoRoutine routine = factory.newRoutine("Depot Cycle");

    // Load the routine's trajectories
    AutoTrajectory depotCycle = routine.trajectory("DepotCycle");

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(Commands.sequence(depotCycle.resetOdometry(), depotCycle.cmd()));

    return routine;
  }

  public AutoRoutine depotInside() {
    AutoRoutine routine = factory.newRoutine("Depot Inside");

    // Load the routine's trajectories
    AutoTrajectory depotCycle = routine.trajectory("DepotInside");

    // When the routine begins, reset odometry and start the first trajectory (1)
    routine.active().onTrue(Commands.sequence(depotCycle.resetOdometry(), depotCycle.cmd()));

    return routine;
  }

  public AutoRoutine outpostDoublePass() {
    return buildDoublePass(
        "Outpost Double Pass", "OutpostBump", "OutpostDoublePass", "OutpostBumpReturn", false);
  }

  public AutoRoutine outpostDoublePassShootFirst() {
    return buildDoublePass(
        "Outpost Double Pass Shoot First",
        "OutpostBump",
        "OutpostDoublePass",
        "OutpostBumpReturn",
        true);
  }

  public AutoRoutine depotDoublePass() {
    return buildDoublePass(
        "Depot Double Pass", "DepotBump", "DepotDoublePass", "DepotBumpReturn", false);
  }

  public AutoRoutine depotDoublePassShootFirst() {
    return buildDoublePass(
        "Depot Double Pass Shoot First", "DepotBump", "DepotDoublePass", "DepotBumpReturn", true);
  }

  public AutoRoutine outpostFullPass() {
    return buildDoublePass(
        "Outpost FULL Pass", "OutpostBump", "FullOutpostSinglePass", "OutpostBumpReturn", false);
  }

  public AutoRoutine outpostSinglePass() {
    return buildDoublePass(
        "Outpost Single Pass", "OutpostBump", "OutpostSinglePass", "OutpostBumpReturn", false);
  }

  public AutoRoutine outpostSinglePassShootFirst() {
    return buildDoublePass(
        "Outpost Single Pass Shoot First",
        "OutpostBump",
        "OutpostSinglePass",
        "OutpostBumpReturn",
        true);
  }

  public AutoRoutine depotSinglePass() {
    return buildDoublePass(
        "Depot Single Pass", "DepotBump", "DepotSinglePass", "DepotBumpReturn", false);
  }

  public AutoRoutine depotSinglePassShootFirst() {
    return buildDoublePass(
        "Depot Single Pass Shoot First", "DepotBump", "DepotSinglePass", "DepotBumpReturn", true);
  }

  public AutoRoutine safe() {
    return buildSafe("Safe", false);
  }

  public AutoRoutine safeShootFirst() {
    return buildSafe("Safe Shoot First", true);
  }

  public AutoRoutine shootOnly() {
    AutoRoutine routine = factory.newRoutine("Shoot Only");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)),
                aimBackAtHub().withTimeout(1.5),
                Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                intake.periodicAutoRehomeCommand().withTimeout(10.0),
                Commands.runOnce(
                    () -> {
                      shooter.setGoal(Shooter.Goal.IDLE);
                      indexer.setGoal(Indexer.Goal.IDLE);
                    })));

    return routine;
  }

  /**
   * Builds a double pass auto routine. Crosses bump, intakes across the field via trajectory,
   * returns over bump, aims at hub, and feeds for remaining time. If shootFirst is true, shoots the
   * preloaded ball before crossing the bump.
   */
  private AutoRoutine buildDoublePass(
      String name,
      String bumpTrajName,
      String doublePassTrajName,
      String bumpReturnTrajName,
      boolean shootFirst) {
    AutoRoutine routine = factory.newRoutine(name);
    AutoTrajectory bump = routine.trajectory(bumpTrajName);
    AutoTrajectory doublePass = routine.trajectory(doublePassTrajName);
    AutoTrajectory bumpReturn = routine.trajectory(bumpReturnTrajName);

    // Build shoot-first prefix if needed
    Command shootFirstSequence =
        shootFirst
            ? Commands.sequence(
                Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)),
                aimBackAtHub().withTimeout(1.5),
                Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                Commands.waitSeconds(1.0),
                Commands.runOnce(
                    () -> {
                      indexer.setGoal(Indexer.Goal.IDLE);
                      shooter.setGoal(Shooter.Goal.IDLE);
                    }))
            : Commands.none();

    routine
        .active()
        .onTrue(
            Commands.sequence(
                // Seed odometry from bump trajectory start
                bump.resetOdometry(),

                // Optionally shoot preloaded ball first
                shootFirstSequence,

                // Deploy intake before crossing bump
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),

                // Cross the bump via Choreo trajectory
                bump.cmd(),

                // PID to double pass trajectory start
                sprintToPose(doublePass.getInitialPose().orElse(new Pose2d())).withTimeout(3.0),

                // Run double pass trajectory
                doublePass.cmd(),

                // Spin up shooter (auto-tracks RPM/hood angle from distance LUT)
                Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)),

                // Retract intake before crossing back over the bump
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.IDLE)),

                // PID to bump return trajectory start
                sprintToPose(bumpReturn.getInitialPose().orElse(new Pose2d())).withTimeout(3.0),

                // Cross back over bump via Choreo trajectory
                bumpReturn.cmd(),

                // Sprint to final shooting position after bump (corrects positional error from bump
                // crossing)
                sprintToPose(bumpReturn.getFinalPose().orElse(new Pose2d())).withTimeout(2.0),

                // Aim at hub continuously while feeding/shooting for remaining time;
                // deadline ends when the feed sequence finishes
                Commands.deadline(
                    Commands.sequence(
                        Commands.waitSeconds(1.0),
                        Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                        intake.periodicAutoRehomeCommand().withTimeout(10.0)),
                    aimBackAtHub()),

                // Cleanup
                Commands.runOnce(
                    () -> {
                      shooter.setGoal(Shooter.Goal.IDLE);
                      indexer.setGoal(Indexer.Goal.IDLE);
                      intake.setGoal(Intake.Goal.IDLE);
                    })));

    return routine;
  }

  /**
   * Builds a safe auto routine. Deploys intake at the start, follows the Safe trajectory, then
   * shoots for remaining time. If shootFirst is true, shoots the preloaded ball before starting.
   */
  private AutoRoutine buildSafe(String name, boolean shootFirst) {
    AutoRoutine routine = factory.newRoutine(name);
    AutoTrajectory safeTraj = routine.trajectory("Safe");

    // Build shoot-first prefix if needed
    Command shootFirstSequence =
        shootFirst
            ? Commands.sequence(
                Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)),
                aimBackAtHub().withTimeout(1.5),
                Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                Commands.waitSeconds(1.0),
                Commands.runOnce(
                    () -> {
                      indexer.setGoal(Indexer.Goal.IDLE);
                      shooter.setGoal(Shooter.Goal.IDLE);
                    }))
            : Commands.none();

    // Spin up shooter on the "Shoot" event marker during trajectory
    safeTraj.atTime("Shoot").onTrue(Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                // Seed odometry from trajectory start
                safeTraj.resetOdometry(),

                // Optionally shoot preloaded ball first
                shootFirstSequence,

                // Deploy intake at beginning of trajectory
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),

                // Follow the Safe trajectory
                safeTraj.cmd(),

                // Fine-tune aim (back at hub) before shooting
                aimBackAtHub().withTimeout(1.0),

                // Feed and shoot for remaining time
                Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                intake.periodicAutoRehomeCommand().withTimeout(10.0),

                // Cleanup
                Commands.runOnce(
                    () -> {
                      shooter.setGoal(Shooter.Goal.IDLE);
                      indexer.setGoal(Indexer.Goal.IDLE);
                      intake.setGoal(Intake.Goal.IDLE);
                    })));

    return routine;
  }

  /** Sprint to a target pose at high speed with heading control. */
  private Command sprintToPose(Pose2d target) {
    ProfiledPIDController headingController =
        new ProfiledPIDController(5.0, 0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Pose2d current = drive.getPose();
              Translation2d toTarget = target.getTranslation().minus(current.getTranslation());
              double distance = toTarget.getNorm();
              Rotation2d direction = toTarget.getAngle();

              // Full speed, decelerate in last 0.5m
              double speed =
                  Math.min(drive.getMaxLinearSpeedMetersPerSec(), Math.max(0.5, distance * 3.0));

              double omega =
                  headingController.calculate(
                      current.getRotation().getRadians(), target.getRotation().getRadians());

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      direction.getCos() * speed,
                      direction.getSin() * speed,
                      omega,
                      current.getRotation()));
            },
            drive)
        .beforeStarting(() -> headingController.reset(drive.getRotation().getRadians()))
        .until(() -> drive.getPose().getTranslation().getDistance(target.getTranslation()) < 0.3);
  }

  /** Rotate in place to aim back of robot at hub. */
  private Command aimBackAtHub() {
    ProfiledPIDController headingController =
        new ProfiledPIDController(5.0, 0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Pose2d current = drive.getPose();
              boolean isRed =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;
              Translation2d hubCenter = FieldConstants.getHubCenter(isRed);

              Translation2d toHub = hubCenter.minus(current.getTranslation());
              double angleToHub = Math.atan2(toHub.getY(), toHub.getX());
              double targetHeading = angleToHub + Math.PI;

              double omega =
                  headingController.calculate(current.getRotation().getRadians(), targetHeading);

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, omega, current.getRotation()));
            },
            drive)
        .beforeStarting(() -> headingController.reset(drive.getRotation().getRadians()));
  }
}
