package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.util.ChoreoAllianceFlipUtil;
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
import frc.robot.util.LoggedTunableNumber;
import java.util.Optional;
import java.util.Set;
import java.util.function.DoubleSupplier;

public class AutoRoutines {
  // How long to feed the indexer when shooting in auto (smaller hopper = shorter shoot)
  private static final LoggedTunableNumber shootDurationSecs =
      new LoggedTunableNumber("Auto/ShootDurationSecs", 10.0);

  // Max speed (m/s) while PIDing to the staging pose during the shoot sequence.
  private static final LoggedTunableNumber trenchStagingSpeedMps =
      new LoggedTunableNumber("Auto/TrenchStagingSpeedMps", 0.5);

  // Staging poses the robot PID drives toward while shooting, positioning it at the trench
  // entrance before the next collection pass. Outpost = bottom wall (low Y), Depot = top wall.
  private static final LoggedTunableNumber trenchOutpostStagingX =
      new LoggedTunableNumber("Auto/TrenchOutpostStagingX", 3.0);
  private static final LoggedTunableNumber trenchOutpostStagingY =
      new LoggedTunableNumber("Auto/TrenchOutpostStagingY", 0.66);
  private static final LoggedTunableNumber trenchDepotStagingX =
      new LoggedTunableNumber("Auto/TrenchDepotStagingX", 3.0);
  private static final LoggedTunableNumber trenchDepotStagingY =
      new LoggedTunableNumber("Auto/TrenchDepotStagingY", 7.55);

  // Extra delay after shoot-first preload shot before starting the trajectory.
  private static final LoggedTunableNumber shootFirstPostDelaySecs =
      new LoggedTunableNumber("Auto/ShootFirstPostDelaySecs", 0.0);

  // Gather clump detection: roller stator amps above this threshold triggers slow-down.
  private static final LoggedTunableNumber gatherClumpCurrentAmps =
      new LoggedTunableNumber("Auto/GatherClumpCurrentAmps", 25.0);
  // Speed cap (m/s) applied when a clump is detected during a gather pass.
  private static final LoggedTunableNumber gatherSlowSpeedMps =
      new LoggedTunableNumber("Auto/GatherSlowSpeedMps", 1.2);

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
   * Trench Outpost Disrupt: Runs TrenchOutpostDisrupt trajectory (rams opponent), deploys intake at
   * "Intake" waypoint, spins up shooter at "Spin" waypoint. After trajectory ends, aims and shoots
   * (stationary or creeping toward Points start based on Auto/TrenchShootOnMove tunable). Then runs
   * TrenchOutpostPoints for a second scoring pass.
   */
  public AutoRoutine trenchOutpostDisrupt() {
    return buildTrenchDisrupt(
        "Trench Outpost Disrupt",
        routine -> routine.trajectory("TrenchOutpostDisrupt"),
        routine -> routine.trajectory("TrenchOutpostPoints"),
        routine -> routine.trajectory("OutpostStagingGather"),
        trenchOutpostStagingX::get,
        trenchOutpostStagingY::get);
  }

  /** Trench Depot Disrupt: Mirrors outpost disrupt to the depot side via mirrorY(). */
  public AutoRoutine trenchDepotDisrupt() {
    return buildTrenchDisrupt(
        "Trench Depot Disrupt",
        routine -> routine.trajectory("TrenchOutpostDisrupt").mirrorY(),
        routine -> routine.trajectory("TrenchOutpostPoints").mirrorY(),
        routine -> routine.trajectory("OutpostStagingGather").mirrorY(),
        trenchDepotStagingX::get,
        trenchDepotStagingY::get);
  }

  /**
   * Trench Outpost Points: Deploys intake immediately, runs TrenchOutpostPoints trajectory. Spins
   * up shooter at "Spin" waypoint. After trajectory ends, aims and shoots (stationary or creeping
   * toward Points start based on Auto/TrenchShootOnMove tunable). Then runs Points again.
   */
  public AutoRoutine trenchOutpostPoints() {
    return buildTrenchPoints(
        "Trench Outpost Points",
        routine -> routine.trajectory("TrenchOutpostPoints"),
        routine -> routine.trajectory("OutpostStagingGather"),
        trenchOutpostStagingX::get,
        trenchOutpostStagingY::get);
  }

  /** Trench Depot Points: Mirrors outpost points to the depot side via mirrorY(). */
  public AutoRoutine trenchDepotPoints() {
    return buildTrenchPoints(
        "Trench Depot Points",
        routine -> routine.trajectory("TrenchOutpostPoints").mirrorY(),
        routine -> routine.trajectory("OutpostStagingGather").mirrorY(),
        trenchDepotStagingX::get,
        trenchDepotStagingY::get);
  }

  /**
   * Trench Outpost Follow: Shoots the preloaded ball first, then follows TrenchOutpostFollow
   * (intake at "Intake", shooter spinup at "Spin"), aims and shoots while staging, then gathers.
   */
  public AutoRoutine trenchOutpostFollow() {
    return buildTrenchFollow(
        "Trench Outpost Follow",
        routine -> routine.trajectory("TrenchOutpostFollow"),
        routine -> routine.trajectory("OutpostStagingGather"),
        trenchOutpostStagingX::get,
        trenchOutpostStagingY::get);
  }

  /** Trench Depot Follow: Mirrors outpost follow to the depot side via mirrorY(). */
  public AutoRoutine trenchDepotFollow() {
    return buildTrenchFollow(
        "Trench Depot Follow",
        routine -> routine.trajectory("TrenchOutpostFollow").mirrorY(),
        routine -> routine.trajectory("OutpostStagingGather").mirrorY(),
        trenchDepotStagingX::get,
        trenchDepotStagingY::get);
  }

  /**
   * Builds a trench disrupt auto. Runs the disrupt trajectory (intake at "Intake" waypoint, shooter
   * spinup at "Spin" waypoint), then aims and shoots, then repeats Points trajectory cycles until
   * auto ends.
   */
  private AutoRoutine buildTrenchDisrupt(
      String name,
      java.util.function.Function<AutoRoutine, AutoTrajectory> disruptFactory,
      java.util.function.Function<AutoRoutine, AutoTrajectory> pointsFactory,
      java.util.function.Function<AutoRoutine, AutoTrajectory> gatherFactory,
      DoubleSupplier stagingX,
      DoubleSupplier stagingY) {
    AutoRoutine routine = factory.newRoutine(name);
    AutoTrajectory disruptTraj = disruptFactory.apply(routine);
    AutoTrajectory pointsTraj = pointsFactory.apply(routine);
    AutoTrajectory gatherTraj = gatherFactory.apply(routine);

    // Deploy intake at the "Intake" waypoint on the disrupt trajectory
    disruptTraj.atTime("Intake").onTrue(Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)));
    // Spin up shooter at the "Spin" waypoint (before crossing bump)
    disruptTraj.atTime("Spin").onTrue(Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                disruptTraj.resetOdometry(),

                // Rush out; intake and shooter activate via waypoint events
                disruptTraj.cmd(),

                // Retract intake for shooting
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.IDLE)),

                // Aim at hub and shoot while PIDing toward staging pose near the wall
                trenchShootSequence(stagingX, stagingY),

                // Gather: run OutpostStagingGather, deploying intake at its "Intake" marker
                trenchGatherRun(gatherTraj)));

    return routine;
  }

  /**
   * Builds a trench follow auto. Shoots preloaded ball first, then runs the follow trajectory
   * (intake at "Intake", shooter spinup at "Spin"), then aims and shoots while staging, then
   * gathers.
   */
  private AutoRoutine buildTrenchFollow(
      String name,
      java.util.function.Function<AutoRoutine, AutoTrajectory> followFactory,
      java.util.function.Function<AutoRoutine, AutoTrajectory> gatherFactory,
      DoubleSupplier stagingX,
      DoubleSupplier stagingY) {
    AutoRoutine routine = factory.newRoutine(name);
    AutoTrajectory followTraj = followFactory.apply(routine);
    AutoTrajectory gatherTraj = gatherFactory.apply(routine);

    followTraj.atTime("Intake").onTrue(Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)));
    followTraj.atTime("Spin").onTrue(Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                followTraj.resetOdometry(),

                // Shoot preloaded ball before driving out
                Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)),
                aimBackAtHub().withTimeout(1.5),
                Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                Commands.waitSeconds(1.0),
                Commands.runOnce(
                    () -> {
                      indexer.setGoal(Indexer.Goal.IDLE);
                      shooter.setGoal(Shooter.Goal.IDLE);
                    }),
                Commands.defer(() -> Commands.waitSeconds(shootFirstPostDelaySecs.get()), Set.of()),

                // Follow trajectory; intake and shooter activate via waypoint events
                followTraj.cmd(),

                // Retract intake for shooting
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.IDLE)),

                // Aim at hub and shoot while PIDing toward staging pose near the wall
                trenchShootSequence(stagingX, stagingY),

                // Gather: run OutpostStagingGather, deploying intake at its "Intake" marker
                trenchGatherRun(gatherTraj)));

    return routine;
  }

  /**
   * Builds a trench points auto. Deploys intake immediately, runs the points trajectory (shooter
   * spinup at "Spin" waypoint), then aims and shoots, then gathers until auto ends.
   */
  private AutoRoutine buildTrenchPoints(
      String name,
      java.util.function.Function<AutoRoutine, AutoTrajectory> pointsFactory,
      java.util.function.Function<AutoRoutine, AutoTrajectory> gatherFactory,
      DoubleSupplier stagingX,
      DoubleSupplier stagingY) {
    AutoRoutine routine = factory.newRoutine(name);
    AutoTrajectory pointsTraj = pointsFactory.apply(routine);
    AutoTrajectory gatherTraj = gatherFactory.apply(routine);

    // Spin up shooter at the "Spin" waypoint (before crossing bump)
    pointsTraj.atTime("Spin").onTrue(Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)));

    routine
        .active()
        .onTrue(
            Commands.sequence(
                pointsTraj.resetOdometry(),

                // Deploy intake right away
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),

                // Follow Points trajectory; shooter spins up at "Spin" waypoint
                pointsTraj.cmd(),

                // Retract intake for shooting
                Commands.runOnce(() -> intake.setGoal(Intake.Goal.IDLE)),

                // Aim at hub and shoot while PIDing toward staging pose near the wall
                trenchShootSequence(stagingX, stagingY),

                // Gather: run OutpostStagingGather, deploying intake at its "Intake" marker
                trenchGatherRun(gatherTraj)));

    return routine;
  }

  /**
   * Runs the gather trajectory once. Intake deploys at its "Intake" waypoint marker. Reactively
   * caps drive speed to Auto/GatherSlowSpeedMps when roller current exceeds
   * Auto/GatherClumpCurrentAmps, allowing the robot to push through dense ball packs without
   * shoving them away.
   */
  private Command trenchGatherRun(AutoTrajectory gatherTraj) {
    gatherTraj.atTime("Intake").onTrue(Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)));
    return Commands.deadline(
            gatherTraj.cmd(),
            Commands.run(
                () -> {
                  if (intake.getRollerStatorCurrentAmps() > gatherClumpCurrentAmps.get()) {
                    drive.setTrajectorySpeedCap(gatherSlowSpeedMps.get());
                  } else {
                    drive.clearTrajectorySpeedCap();
                  }
                }))
        .finallyDo(interrupted -> drive.clearTrajectorySpeedCap());
  }

  /**
   * Aims the back of the robot at the hub and feeds. Simultaneously PID drives toward the tunable
   * staging pose (Auto/TrenchStagingX/Y/Deg) near the side wall so the robot arrives at the trench
   * entrance by the time shooting finishes. Heading and translation are decoupled.
   */
  private Command trenchShootSequence(DoubleSupplier stagingX, DoubleSupplier stagingY) {
    ProfiledPIDController headingController =
        new ProfiledPIDController(5.0, 0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    Command aimAndDrive =
        Commands.run(
                () -> {
                  Pose2d current = drive.getPose();

                  // Heading: back of robot aimed at compensated virtual target (accounts for
                  // robot velocity so lateral drift during motion is corrected).
                  Translation2d aimTarget = shooter.getAimTarget();
                  Translation2d toTarget = aimTarget.minus(current.getTranslation());
                  double angleToTarget = Math.atan2(toTarget.getY(), toTarget.getX());
                  double targetHeading =
                      angleToTarget
                          + Math.PI
                          + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);
                  double omega =
                      headingController.calculate(
                          current.getRotation().getRadians(), targetHeading);

                  // Translation: proportional drive toward staging pose near the wall.
                  // Flip for red alliance using the same transform Choreo applies to trajectories.
                  Translation2d staging =
                      new Translation2d(stagingX.getAsDouble(), stagingY.getAsDouble());
                  if (ChoreoAllianceFlipUtil.shouldFlip()) {
                    staging = ChoreoAllianceFlipUtil.flip(staging);
                  }
                  Translation2d toStaging = staging.minus(current.getTranslation());
                  double distance = toStaging.getNorm();
                  double vx = 0;
                  double vy = 0;
                  if (distance > 0.15) {
                    double angle = Math.atan2(toStaging.getY(), toStaging.getX());
                    double speed = Math.min(trenchStagingSpeedMps.get(), distance * 2.0);
                    vx = Math.cos(angle) * speed;
                    vy = Math.sin(angle) * speed;
                  }

                  drive.runVelocity(
                      ChassisSpeeds.fromFieldRelativeSpeeds(vx, vy, omega, current.getRotation()));
                },
                drive)
            .beforeStarting(() -> headingController.reset(drive.getRotation().getRadians()));

    return Commands.deadline(
        Commands.sequence(
            Commands.waitUntil(() -> shooter.isAtSetpoint()).withTimeout(2.0),
            Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
            Commands.defer(() -> Commands.waitSeconds(shootDurationSecs.get()), Set.of()),
            Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.IDLE))),
        aimAndDrive,
        intake.periodicAutoRehomeCommand());
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
                    }),
                Commands.defer(() -> Commands.waitSeconds(shootFirstPostDelaySecs.get()), Set.of()))
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

                // Aim at hub and shoot for configured duration
                Commands.deadline(
                    Commands.sequence(
                        Commands.waitUntil(
                                () -> {
                                  boolean isRed =
                                      DriverStation.getAlliance().isPresent()
                                          && DriverStation.getAlliance().get() == Alliance.Red;
                                  Translation2d toHub =
                                      FieldConstants.getHubCenter(isRed)
                                          .minus(drive.getPose().getTranslation());
                                  double targetHeading =
                                      Math.atan2(toHub.getY(), toHub.getX())
                                          + Math.PI
                                          + Math.toRadians(
                                              ShooterConstants.shooterHeadingOffsetDegrees);
                                  return shooter.isAtSetpoint()
                                      && drive.isHeadingAt(
                                          Rotation2d.fromRadians(targetHeading), 3.0);
                                })
                            .withTimeout(1.5),
                        Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                        Commands.defer(
                            () -> Commands.waitSeconds(shootDurationSecs.get()), Set.of()),
                        Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.IDLE))),
                    aimBackAtHub(),
                    intake.periodicAutoRehomeCommand()),

                // Continue cycling: deploy intake, cross bump, collect, return, shoot — repeats
                // until auto ends
                Commands.sequence(
                        Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)),
                        bump.cmd(),
                        sprintToPose(doublePass.getInitialPose().orElse(new Pose2d()))
                            .withTimeout(3.0),
                        doublePass.cmd(),
                        Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)),
                        Commands.runOnce(() -> intake.setGoal(Intake.Goal.IDLE)),
                        sprintToPose(bumpReturn.getInitialPose().orElse(new Pose2d()))
                            .withTimeout(3.0),
                        bumpReturn.cmd(),
                        sprintToPose(bumpReturn.getFinalPose().orElse(new Pose2d()))
                            .withTimeout(2.0),
                        Commands.deadline(
                            Commands.sequence(
                                Commands.waitUntil(
                                        () -> {
                                          boolean isRed =
                                              DriverStation.getAlliance().isPresent()
                                                  && DriverStation.getAlliance().get()
                                                      == Alliance.Red;
                                          Translation2d toHub =
                                              FieldConstants.getHubCenter(isRed)
                                                  .minus(drive.getPose().getTranslation());
                                          double targetHeading =
                                              Math.atan2(toHub.getY(), toHub.getX())
                                                  + Math.PI
                                                  + Math.toRadians(
                                                      ShooterConstants.shooterHeadingOffsetDegrees);
                                          return shooter.isAtSetpoint()
                                              && drive.isHeadingAt(
                                                  Rotation2d.fromRadians(targetHeading), 3.0);
                                        })
                                    .withTimeout(1.5),
                                Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.FEED)),
                                Commands.defer(
                                    () -> Commands.waitSeconds(shootDurationSecs.get()), Set.of()),
                                Commands.runOnce(() -> indexer.setGoal(Indexer.Goal.IDLE))),
                            aimBackAtHub()))
                    .repeatedly()));

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
                    }),
                Commands.defer(() -> Commands.waitSeconds(shootFirstPostDelaySecs.get()), Set.of()))
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
    return sprintToPose(target, 0.3);
  }

  /**
   * Sprint to a target pose. Exits when within {@code exitDistanceMeters} of the target, allowing a
   * larger value to hand off to a trajectory while still at speed.
   */
  private Command sprintToPose(Pose2d target, double exitDistanceMeters) {
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
        .until(
            () ->
                drive.getPose().getTranslation().getDistance(target.getTranslation())
                    < exitDistanceMeters);
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
                        drive.getMaxLinearSpeedMetersPerSec(),
                        Math.max(0.5, Math.abs(yError) * 3.0));
                vy = Math.signum(yError) * speed;
                vx = 0;
              } else {
                // Phase 2: drive in X direction
                double speed =
                    Math.min(
                        drive.getMaxLinearSpeedMetersPerSec(),
                        Math.max(0.5, Math.abs(xError) * 3.0));
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
