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
import frc.robot.subsystems.shooter.ShooterConstants;
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

  /**
   * Depot Single Pass - Shoot On Move variant. Crosses bump out, collects balls on the single pass
   * trajectory, returns over the bump, then shoots while rolling toward the depot at 0.5 m/s from
   * the alliance side. Deploys intake when ~2m from the depot to collect staged balls while still
   * firing. Runs until auto ends.
   */
  public AutoRoutine depotSinglePassShootOnMove() {
    AutoRoutine routine = factory.newRoutine("Depot Single Pass Shoot On Move");
    AutoTrajectory bump = routine.trajectory("DepotBump");
    AutoTrajectory singlePass = routine.trajectory("DepotSinglePass");
    AutoTrajectory bumpReturn = routine.trajectory("DepotBumpReturn");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                // Seed odometry from bump trajectory start
                bump.resetOdometry(),

                // Deploy intake before crossing bump
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),

                // Cross the bump via Choreo trajectory
                bump.cmd(),

                // PID to single pass trajectory start
                sprintToPose(singlePass.getInitialPose().orElse(new Pose2d())).withTimeout(3.0),

                // Run single pass trajectory with intake collecting balls
                singlePass.cmd(),

                // Retract intake, spin up shooter, and return over the bump.
                // The shooter has the full bump-return transit time to reach setpoint.
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.IDLE)),
                Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)),
                bumpReturn.cmd(),

                // Correct positional error introduced by bump crossing (same as Shoot First auto)
                sprintToPose(bumpReturn.getFinalPose().orElse(new Pose2d())).withTimeout(2.0),

                // Now on the alliance side: drive toward depot at 0.5 m/s while shooting.
                // Three branches run in parallel until auto ends:
                //   1. Drive + heading: translate toward depot approach target, back aimed at hub
                //   2. Feed: wait for shooter setpoint, then own the indexer in FEED mode
                //   3. Intake: jostle balls (periodic rehome) until within ~2m of the depot,
                //      then deploy to collect staged balls -- mutually exclusive with jostling
                Commands.parallel(
                    driveTowardDepotAimingAtHub(0.5),
                    Commands.sequence(
                        Commands.waitUntil(() -> shooter.isAtSetpoint()), indexer.feedCommand()),
                    Commands.sequence(
                        intake
                            .periodicAutoRehomeCommand()
                            .until(
                                () -> {
                                  boolean isRed =
                                      DriverStation.getAlliance().isPresent()
                                          && DriverStation.getAlliance().get() == Alliance.Red;
                                  return drive
                                          .getPose()
                                          .getTranslation()
                                          .getDistance(FieldConstants.getDepotCenter(isRed))
                                      < 2.0;
                                }),
                        Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE))))));

    return routine;
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
   * Trench Outpost Disrupt: Drives out to the trench via TrenchOutpostDisrupt, then returns to the
   * starting spot while shooting (Y-direction first, then X), then runs TrenchOutpostExit for a
   * second pass to a shooting position.
   */
  public AutoRoutine trenchOutpostDisrupt() {
    AutoRoutine routine = factory.newRoutine("Trench Outpost Disrupt");
    AutoTrajectory disruptTraj = routine.trajectory("TrenchOutpostDisrupt");
    AutoTrajectory exitTraj = routine.trajectory("TrenchOutpostExit");

    routine
        .active()
        .onTrue(
            Commands.sequence(
                // Seed odometry from disrupt trajectory start
                disruptTraj.resetOdometry(),

                // Go disrupt the trench (no intake)
                disruptTraj.cmd(),

                // Spin up shooter for return journey
                Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)),

                // Shoot while driving back to starting spot (Y-direction first, then X)
                Commands.parallel(
                        driveYThenXAimingAtHub(exitTraj.getInitialPose().orElse(new Pose2d())),
                        Commands.sequence(
                            Commands.waitUntil(() -> shooter.isAtSetpoint()),
                            indexer.feedCommand()),
                        intake.periodicAutoRehomeCommand())
                    .withTimeout(5.0),

                // Fine-tune to exact TrenchOutpostExit start
                sprintToPose(exitTraj.getInitialPose().orElse(new Pose2d())).withTimeout(2.0),

                // Run TrenchOutpostExit for second pass
                exitTraj.cmd(),

                // Aim at hub and feed for remaining time
                Commands.deadline(
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                        intake.periodicAutoRehomeCommand().withTimeout(8.0)),
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
   * Trench Outpost Rush: Drives out to the trench via TrenchOutpostDisrupt, deploying intake after
   * reaching the first waypoint to collect pieces. Returns to starting spot while shooting
   * (Y-direction first, then X), then runs TrenchOutpostExit for a second pass.
   */
  public AutoRoutine trenchOutpostRush() {
    AutoRoutine routine = factory.newRoutine("Trench Outpost Rush");
    AutoTrajectory disruptTraj = routine.trajectory("TrenchOutpostDisrupt");
    AutoTrajectory exitTraj = routine.trajectory("TrenchOutpostExit");

    // Deploy intake when the robot reaches the first intermediate waypoint (~1.25s in)
    disruptTraj
        .atTime(1.25256)
        .onTrue(Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                // Seed odometry from disrupt trajectory start
                disruptTraj.resetOdometry(),

                // Rush into trench; intake deploys via event trigger at first waypoint
                disruptTraj.cmd(),

                // Retract intake, spin up shooter for return
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.IDLE)),
                Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)),

                // Shoot while driving back to starting spot (Y-direction first, then X)
                Commands.parallel(
                        driveYThenXAimingAtHub(exitTraj.getInitialPose().orElse(new Pose2d())),
                        Commands.sequence(
                            Commands.waitUntil(() -> shooter.isAtSetpoint()),
                            indexer.feedCommand()),
                        intake.periodicAutoRehomeCommand())
                    .withTimeout(5.0),

                // Fine-tune to exact TrenchOutpostExit start
                sprintToPose(exitTraj.getInitialPose().orElse(new Pose2d())).withTimeout(2.0),

                // Run TrenchOutpostExit for second pass
                exitTraj.cmd(),

                // Aim at hub and feed for remaining time
                Commands.deadline(
                    Commands.sequence(
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                        intake.periodicAutoRehomeCommand().withTimeout(8.0)),
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

  /**
   * Translate toward the depot-side fuel pool center at {@code speedMps} while the heading PID
   * independently keeps the back of the robot aimed at the hub. Translation and rotation are
   * decoupled, so shoot-on-move compensation in the shooter handles the moving shot correctly.
   */
  private Command driveTowardDepotAimingAtHub(double speedMps) {
    ProfiledPIDController headingController =
        new ProfiledPIDController(5.0, 0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Pose2d current = drive.getPose();
              boolean isRed =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              // Rotation: keep back of robot aimed at hub
              Translation2d hubCenter = FieldConstants.getHubCenter(isRed);
              Translation2d toHub = hubCenter.minus(current.getTranslation());
              double angleToHub = Math.atan2(toHub.getY(), toHub.getX());
              double targetHeading =
                  angleToHub
                      + Math.PI
                      + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);
              double omega =
                  headingController.calculate(current.getRotation().getRadians(), targetHeading);

              // Translation: drive toward depot
              Translation2d depotTarget = FieldConstants.getDepotApproachTarget(isRed);
              Translation2d toDepot = depotTarget.minus(current.getTranslation());
              double driveAngle = Math.atan2(toDepot.getY(), toDepot.getX());

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(
                      Math.cos(driveAngle) * speedMps,
                      Math.sin(driveAngle) * speedMps,
                      omega,
                      current.getRotation()));
            },
            drive)
        .beforeStarting(() -> headingController.reset(drive.getRotation().getRadians()));
  }

  /**
   * Drive to {@code target} in two phases while aiming the back of the robot at the hub: first
   * travel in the Y direction until within 0.3m of the target Y, then travel in the X direction
   * until within 0.3m of the target overall. Decelerates proportionally as in sprintToPose.
   */
  private Command driveYThenXAimingAtHub(Pose2d target) {
    ProfiledPIDController headingController =
        new ProfiledPIDController(5.0, 0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    return Commands.run(
            () -> {
              Pose2d current = drive.getPose();
              boolean isRed =
                  DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red;

              // Heading: keep back of robot aimed at hub
              Translation2d hubCenter = FieldConstants.getHubCenter(isRed);
              Translation2d toHub = hubCenter.minus(current.getTranslation());
              double angleToHub = Math.atan2(toHub.getY(), toHub.getX());
              double targetHeading =
                  angleToHub
                      + Math.PI
                      + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);
              double omega =
                  headingController.calculate(current.getRotation().getRadians(), targetHeading);

              // Translation: Y-first then X
              double yError = target.getY() - current.getY();
              double xError = target.getX() - current.getX();

              double vx, vy;
              if (Math.abs(yError) > 0.3) {
                // Phase 1: drive in Y direction
                double speed =
                    Math.min(
                        drive.getMaxLinearSpeedMetersPerSec(), Math.max(0.5, Math.abs(yError) * 3.0));
                vy = Math.signum(yError) * speed;
                vx = 0;
              } else {
                // Phase 2: drive in X direction
                double speed =
                    Math.min(
                        drive.getMaxLinearSpeedMetersPerSec(), Math.max(0.5, Math.abs(xError) * 3.0));
                vx = Math.signum(xError) * speed;
                vy = 0;
              }

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, current.getRotation()));
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
              double targetHeading =
                  angleToHub
                      + Math.PI
                      + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);

              double omega =
                  headingController.calculate(current.getRotation().getRadians(), targetHeading);

              drive.runVelocity(
                  ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, omega, current.getRotation()));
            },
            drive)
        .beforeStarting(() -> headingController.reset(drive.getRotation().getRadians()));
  }
}
