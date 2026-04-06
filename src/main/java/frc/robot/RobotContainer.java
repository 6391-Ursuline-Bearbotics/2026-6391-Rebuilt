// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerBeltIO;
import frc.robot.subsystems.indexer.IndexerBeltIOSim;
import frc.robot.subsystems.indexer.IndexerBeltIOTalonFX;
import frc.robot.subsystems.indexer.IndexerKickerIO;
import frc.robot.subsystems.indexer.IndexerKickerIOSim;
import frc.robot.subsystems.indexer.IndexerKickerIOTalonFX;
import frc.robot.subsystems.indexer.SpinnersIO;
import frc.robot.subsystems.indexer.SpinnersIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeDeployIO;
import frc.robot.subsystems.intake.IntakeDeployIOSim;
import frc.robot.subsystems.intake.IntakeDeployIOTalonFX;
import frc.robot.subsystems.intake.IntakeRollerIO;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.shooter.ShooterHoodIO;
import frc.robot.subsystems.shooter.ShooterHoodIOServo;
import frc.robot.subsystems.shooter.ShooterHoodIOSim;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Drive modes
  public enum DriveMode {
    STANDARD,
    SNAKE,
    AIM_TARGET // Aims at Hub in alliance zone, passing target otherwise
  }

  // Subsystems
  private final Drive drive;
  private final Vision vision;
  private final Intake intake;
  private final Indexer indexer;
  private final Shooter shooter;

  // Set to false to use no-op IO when hardware is not connected
  private static final boolean indexerEnabled = true;
  private static final boolean shooterEnabled = true;

  // Controllers
  private final CommandXboxController drv = new CommandXboxController(0);
  private final CommandXboxController op = new CommandXboxController(1);

  // Auto
  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser;

  // Current drive mode
  private DriveMode currentDriveMode = DriveMode.STANDARD;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));

        // Vision with 1 Limelight (camera0 only for initial testing)
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation));

        // Intake with TalonFX hardware
        intake = new Intake(new IntakeDeployIOTalonFX(), new IntakeRollerIOTalonFX());

        // Indexer with TalonFX hardware (no-op IO when disabled)
        indexer =
            indexerEnabled
                ? new Indexer(
                    new IndexerBeltIOTalonFX(),
                    new IndexerKickerIOTalonFX(),
                    new SpinnersIOSparkMax(),
                    drive::getPose)
                : new Indexer(
                    new IndexerBeltIO() {},
                    new IndexerKickerIO() {},
                    new SpinnersIO() {},
                    drive::getPose);

        // Shooter with TalonFX hardware (no-op IO when disabled)
        shooter =
            shooterEnabled
                ? new Shooter(
                    new ShooterIOTalonFX(),
                    new ShooterHoodIOServo(),
                    drive::getPose,
                    drive::getFieldRelativeSpeeds,
                    () -> indexer.getGoal() == Indexer.Goal.FEED,
                    drive::getPitch)
                : new Shooter(
                    new ShooterIO() {},
                    new ShooterHoodIO() {},
                    drive::getPose,
                    drive::getFieldRelativeSpeeds,
                    () -> indexer.getGoal() == Indexer.Goal.FEED,
                    drive::getPitch);

        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose));
        intake = new Intake(new IntakeDeployIOSim(), new IntakeRollerIOSim());
        indexer =
            new Indexer(
                new IndexerBeltIOSim(),
                new IndexerKickerIOSim(),
                new SpinnersIO() {},
                drive::getPose);
        shooter =
            new Shooter(
                new ShooterIOSim(),
                new ShooterHoodIOSim(),
                drive::getPose,
                drive::getFieldRelativeSpeeds,
                () -> indexer.getGoal() == Indexer.Goal.FEED,
                drive::getPitch);
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        intake = new Intake(new IntakeDeployIO() {}, new IntakeRollerIO() {});
        indexer =
            new Indexer(
                new IndexerBeltIO() {},
                new IndexerKickerIO() {},
                new SpinnersIO() {},
                drive::getPose);
        shooter =
            new Shooter(
                new ShooterIO() {},
                new ShooterHoodIO() {},
                drive::getPose,
                drive::getFieldRelativeSpeeds,
                () -> indexer.getGoal() == Indexer.Goal.FEED,
                drive::getPitch);
        break;
    }

    // Set up Choreo auto factory, routines, and chooser
    autoFactory =
        new AutoFactory(drive::getPose, drive::setPose, drive::followTrajectory, true, drive);
    autoRoutines = new AutoRoutines(autoFactory, drive, intake, indexer, shooter);
    autoChooser = new AutoChooser();
    SmartDashboard.putData("Auto Choices", autoChooser);

    // Competition auto routines (always available)
    autoChooser.addRoutine("Depot Double Pass", autoRoutines::depotDoublePass);
    autoChooser.addRoutine(
        "Depot Double Pass (Shoot First)", autoRoutines::depotDoublePassShootFirst);
    autoChooser.addRoutine("Depot Single Pass", autoRoutines::depotSinglePass);
    autoChooser.addRoutine(
        "Depot Single Pass (Shoot First)", autoRoutines::depotSinglePassShootFirst);
    autoChooser.addRoutine(
        "Depot Single Pass Shoot On Move", autoRoutines::depotSinglePassShootOnMove);
    autoChooser.addRoutine("Outpost Double Pass", autoRoutines::outpostDoublePass);
    autoChooser.addRoutine(
        "Outpost Double Pass (Shoot First)", autoRoutines::outpostDoublePassShootFirst);
    autoChooser.addRoutine("Outpost FULL Pass", autoRoutines::outpostFullPass);
    autoChooser.addRoutine("Outpost Single Pass", autoRoutines::outpostSinglePass);
    autoChooser.addRoutine(
        "Outpost Single Pass (Shoot First)", autoRoutines::outpostSinglePassShootFirst);
    autoChooser.addRoutine("Trench Outpost Disrupt", autoRoutines::trenchOutpostDisrupt);
    autoChooser.addRoutine("Trench Outpost Rush", autoRoutines::trenchOutpostRush);
    autoChooser.addRoutine("Safe", autoRoutines::safe);
    autoChooser.addRoutine("Safe (Shoot First)", autoRoutines::safeShootFirst);
    autoChooser.addRoutine("Shoot Only", autoRoutines::shootOnly);

    // Test and characterization routines (tuning mode only)
    if (Constants.tuningMode) {
      autoChooser.addRoutine("Reset Odometry Start", autoRoutines::resetOdometryStart);
      autoChooser.addRoutine("Depot Cycle", autoRoutines::depotCycle);
      autoChooser.addRoutine("Depot Inside", autoRoutines::depotInside);
      autoChooser.addCmd(
          "Drive Wheel Radius Characterization",
          () -> DriveCommands.wheelRadiusCharacterization(drive));
      autoChooser.addCmd(
          "Drive Simple FF Characterization",
          () -> DriveCommands.feedforwardCharacterization(drive));
      autoChooser.addCmd(
          "Drive Forward 15ft", () -> DriveCommands.driveForward(drive, Units.feetToMeters(15)));
      autoChooser.addCmd(
          "Drive SysId (Quasistatic Forward)",
          () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      autoChooser.addCmd(
          "Drive SysId (Quasistatic Reverse)",
          () -> drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      autoChooser.addCmd(
          "Drive SysId (Dynamic Forward)",
          () -> drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
      autoChooser.addCmd(
          "Drive SysId (Dynamic Reverse)",
          () -> drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
      autoChooser.addCmd(
          "Intake Roller FF Characterization", () -> Intake.rollerFFCharacterization(intake));
      autoChooser.addCmd(
          "Shooter FF Characterization", () -> Shooter.shooterFFCharacterization(shooter));
    }

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command with mode switching
    drive.setDefaultCommand(
        Commands.run(
            () -> {
              switch (currentDriveMode) {
                case STANDARD:
                  runStandardDrive();
                  break;
                case SNAKE:
                  runSnakeDrive();
                  break;
                case AIM_TARGET:
                  runAimTargetDrive();
                  break;
              }
            },
            drive));

    // Mode selection buttons
    drv.b().onTrue(Commands.runOnce(() -> currentDriveMode = DriveMode.STANDARD));
    drv.y()
        .onTrue(
            Commands.runOnce(
                () -> {
                  currentDriveMode = DriveMode.SNAKE;
                  snakeAngleController.reset(drive.getRotation().getRadians());
                  lastSnakeHeading = drive.getRotation().getRadians();
                }));
    drv.a()
        .onTrue(
            Commands.runOnce(
                () -> {
                  currentDriveMode = DriveMode.AIM_TARGET;
                  aimTargetController.reset(drive.getRotation().getRadians());
                  shooter.setGoal(Shooter.Goal.SHOOT);
                }));

    // Aim at hub then lock wheels in X pattern when X is pressed (also spins up shooter)
    drv.x()
        .onTrue(
            Commands.sequence(
                Commands.runOnce(() -> shooter.setGoal(Shooter.Goal.SHOOT)),
                aimAtHub().withTimeout(1.0),
                Commands.runOnce(() -> drive.stopWithX(), drive)));

    // Reset gyro when Start button is pressed
    drv.start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.kZero)),
                    drive)
                .ignoringDisable(true));

    // Operator intake controls
    op.a().onTrue(Commands.runOnce(() -> intake.setGoal(Intake.Goal.INTAKE)));
    op.b().onTrue(Commands.runOnce(() -> intake.setGoal(Intake.Goal.IDLE)));
    op.y().onTrue(Commands.runOnce(() -> intake.setGoal(Intake.Goal.DEPLOYED_IDLE)));

    // Driver clump intake mode (left trigger held)
    drv.leftTrigger(0.5)
        .whileTrue(
            Commands.startEnd(
                () -> {
                  intake.setGoal(Intake.Goal.CLUMP_INTAKE);
                  drive.setMaxSpeedOverride(2.0);
                },
                () -> {
                  intake.setGoal(Intake.Goal.INTAKE);
                  drive.clearMaxSpeedOverride();
                }));

    // Operator indexer controls (ungated feed + auto-spinup + gyration + intake rehome)
    op.leftTrigger(0.5)
        .whileTrue(
            Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          shooter.setGoal(Shooter.Goal.SHOOT);
                        }),
                    Commands.run(() -> indexer.setGoal(Indexer.Goal.FEED))
                        .finallyDo(() -> indexer.setGoal(Indexer.Goal.IDLE)))
                .finallyDo(() -> autoAimGyrating = false)
                .alongWith(intake.periodicAutoRehomeCommand()));

    // Left bumper: Gated auto shot with intake deployed + running (hold to shoot, release to
    // retract)
    op.leftBumper()
        .whileTrue(
            Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          shooter.setGoal(Shooter.Goal.SHOOT);
                          currentDriveMode = DriveMode.AIM_TARGET;
                          aimTargetController.reset(drive.getRotation().getRadians());
                          intake.setGoal(Intake.Goal.INTAKE);
                        }),
                    Commands.waitUntil(() -> shooter.isAtSetpoint() && isAimedAtTarget()),
                    Commands.run(() -> indexer.setGoal(Indexer.Goal.FEED))
                        .finallyDo(() -> indexer.setGoal(Indexer.Goal.IDLE)))
                .finallyDo(
                    () -> {
                      currentDriveMode = DriveMode.STANDARD;
                      autoAimGyrating = false;
                      intake.setGoal(Intake.Goal.IDLE);
                    }));

    // Right bumper: Stop shooter (toggle off) + return to standard drive mode
    op.rightBumper()
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.setGoal(Shooter.Goal.IDLE);
                  currentDriveMode = DriveMode.STANDARD;
                }));

    // Auto-spinup + auto-aim, wait for both, then feed (hold).  Works on both controllers
    op.rightTrigger(0.5)
        .or(drv.rightTrigger(0.5))
        .whileTrue(
            Commands.sequence(
                    Commands.runOnce(
                        () -> {
                          shooter.setGoal(Shooter.Goal.SHOOT);
                          currentDriveMode = DriveMode.AIM_TARGET;
                          aimTargetController.reset(drive.getRotation().getRadians());
                        }),
                    Commands.waitUntil(() -> shooter.isAtSetpoint() && isAimedAtTarget()),
                    Commands.run(() -> indexer.setGoal(Indexer.Goal.FEED))
                        .finallyDo(() -> indexer.setGoal(Indexer.Goal.IDLE)))
                .finallyDo(
                    () -> {
                      currentDriveMode = DriveMode.STANDARD;
                      autoAimGyrating = false;
                    })
                .alongWith(intake.periodicAutoRehomeCommand()));

    // DPAD: Manual distance setpoint + spin up shooter
    // Left = 5ft, Right = 10ft, Up = +0.5ft, Down = -0.5ft
    op.pov(270)
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.setDistanceSetpoint(Units.feetToMeters(5.0));
                  shooter.setGoal(Shooter.Goal.SHOOT);
                }));
    op.pov(90)
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.setDistanceSetpoint(Units.feetToMeters(10.0));
                  shooter.setGoal(Shooter.Goal.SHOOT);
                }));
    op.pov(0)
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.adjustDistanceSetpoint(Units.feetToMeters(0.5));
                  shooter.setGoal(Shooter.Goal.SHOOT);
                }));
    op.pov(180)
        .onTrue(
            Commands.runOnce(
                () -> {
                  shooter.adjustDistanceSetpoint(Units.feetToMeters(-0.5));
                  shooter.setGoal(Shooter.Goal.SHOOT);
                }));

    // Operator X: Eject (reverse indexer + deploy intake in reverse, restore previous state on
    // release)
    op.x()
        .whileTrue(
            Commands.startEnd(
                    () -> {
                      intakeGoalBeforeEject = intake.getGoal();
                      intake.setGoal(Intake.Goal.EJECT);
                    },
                    () -> intake.setGoal(intakeGoalBeforeEject))
                .alongWith(indexer.ejectCommand()));

    // Rumble both controllers 2 seconds before our hub's active shift starts
    new Trigger(() -> GameData.isHubActivatingSoon(5.0))
        .whileTrue(
            Commands.startEnd(
                () -> {
                  drv.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
                  op.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 1.0);
                },
                () -> {
                  drv.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
                  op.getHID().setRumble(GenericHID.RumbleType.kBothRumble, 0.0);
                }));

    // Driver DPAD: snap intake (front) to cardinal field directions, releases rotation once arrived
    // UP = 180°, RIGHT = 90°, DOWN = 0°, LEFT = -90°
    final double kDpadSnapTolerance = Math.toRadians(2.0);
    drv.pov(0) // Up → 180° (intake faces up-field)
        .onTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -drv.getLeftY(),
                    () -> -drv.getLeftX(),
                    () -> Rotation2d.fromDegrees(180))
                .until(
                    () ->
                        Math.abs(
                                drive.getRotation().minus(Rotation2d.fromDegrees(180)).getRadians())
                            < kDpadSnapTolerance));
    drv.pov(90) // Right → 90° (intake faces right)
        .onTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -drv.getLeftY(),
                    () -> -drv.getLeftX(),
                    () -> Rotation2d.fromDegrees(90))
                .until(
                    () ->
                        Math.abs(drive.getRotation().minus(Rotation2d.fromDegrees(90)).getRadians())
                            < kDpadSnapTolerance));
    drv.pov(180) // Down → 0° (intake faces down-field)
        .onTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive, () -> -drv.getLeftY(), () -> -drv.getLeftX(), () -> Rotation2d.kZero)
                .until(
                    () ->
                        Math.abs(drive.getRotation().minus(Rotation2d.kZero).getRadians())
                            < kDpadSnapTolerance));
    drv.pov(270) // Left → -90° (intake faces left)
        .onTrue(
            DriveCommands.joystickDriveAtAngle(
                    drive,
                    () -> -drv.getLeftY(),
                    () -> -drv.getLeftX(),
                    () -> Rotation2d.fromDegrees(-90))
                .until(
                    () ->
                        Math.abs(
                                drive.getRotation().minus(Rotation2d.fromDegrees(-90)).getRadians())
                            < kDpadSnapTolerance));
  }

  // Intake goal saved before an eject so it can be restored on release
  private Intake.Goal intakeGoalBeforeEject = Intake.Goal.IDLE;

  // Lateral gyration during auto-aim shoot
  private boolean autoAimGyrating = false;
  private static final LoggedTunableNumber gyrationAmplitudeInches =
      new LoggedTunableNumber("Shooter/GyrationAmplitudeInches", 0.50);
  private static final LoggedTunableNumber gyrationFreqHz =
      new LoggedTunableNumber("Shooter/GyrationFreqHz", 5.0);

  // Drive mode helper methods
  private final ProfiledPIDController snakeAngleController =
      new ProfiledPIDController(5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(6.0, 15.0));
  private final ProfiledPIDController aimTargetController =
      new ProfiledPIDController(5.0, 0.0, 0.4, new TrapezoidProfile.Constraints(8.0, 20.0));
  private double lastSnakeHeading = 0.0;

  {
    snakeAngleController.enableContinuousInput(-Math.PI, Math.PI);
    aimTargetController.enableContinuousInput(-Math.PI, Math.PI);
  }

  private void runStandardDrive() {
    Translation2d linearVelocity = getLinearVelocityFromJoysticks();
    double omega = MathUtil.applyDeadband(-drv.getRightX(), 0.1);
    omega = Math.copySign(omega * omega, omega);

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega * drive.getMaxAngularSpeedRadPerSec());

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    ChassisSpeeds robotRelative =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation());

    // Add sinusoidal lateral gyration while shoot trigger is held
    if (autoAimGyrating) {
      double ampM = Units.inchesToMeters(gyrationAmplitudeInches.get());
      double freqHz = gyrationFreqHz.get();
      robotRelative.vyMetersPerSecond +=
          ampM
              * 2.0
              * Math.PI
              * freqHz
              * Math.cos(2.0 * Math.PI * freqHz * Timer.getFPGATimestamp());
    }

    drive.runVelocity(robotRelative);
  }

  private void runSnakeDrive() {
    Translation2d linearVelocity = getLinearVelocityFromJoysticks();

    double targetHeading;
    if (linearVelocity.getNorm() > 0.01) {
      targetHeading = Math.atan2(linearVelocity.getY(), linearVelocity.getX());
      lastSnakeHeading = targetHeading;
    } else {
      targetHeading = lastSnakeHeading;
    }

    double omega = snakeAngleController.calculate(drive.getRotation().getRadians(), targetHeading);

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega);

    boolean isFlipped =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
  }

  private void runAimTargetDrive() {
    Translation2d linearVelocity = getLinearVelocityFromJoysticks();

    // Determine alliance
    boolean isRedAlliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;

    // Get robot position on field
    Translation2d robotPosition = drive.getPose().getTranslation();

    // Determine target based on position
    Translation2d target;
    if (FieldConstants.isInOwnAllianceZone(robotPosition, isRedAlliance)) {
      // In our alliance zone - aim at our Hub
      target = FieldConstants.getHubCenter(isRedAlliance);
    } else {
      // In neutral zone or opponent zone - aim at passing target
      target = FieldConstants.getPassingTarget(robotPosition, isRedAlliance);
    }

    // Calculate angle to target — add π so the back (shooter) faces the target
    Translation2d robotToTarget = target.minus(robotPosition);
    double targetHeading =
        Math.atan2(robotToTarget.getY(), robotToTarget.getX())
            + Math.PI
            + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);

    double omega = aimTargetController.calculate(drive.getRotation().getRadians(), targetHeading);

    // Limit translational speed while aiming to improve accuracy
    double aimMaxSpeed = 1.5; // m/s
    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * aimMaxSpeed, linearVelocity.getY() * aimMaxSpeed, omega);

    Rotation2d robotHeading =
        isRedAlliance ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation();
    ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotHeading);

    // Add sinusoidal lateral gyration while shoot trigger is held
    if (autoAimGyrating) {
      double ampM = Units.inchesToMeters(gyrationAmplitudeInches.get());
      double freqHz = gyrationFreqHz.get();
      robotRelative.vyMetersPerSecond +=
          ampM
              * 2.0
              * Math.PI
              * freqHz
              * Math.cos(2.0 * Math.PI * freqHz * Timer.getFPGATimestamp());
    }

    drive.runVelocity(robotRelative);
  }

  private static final double kAimToleranceRad = Math.toRadians(4.5);

  /**
   * Returns true when the robot's shooter is pointed within tolerance of the current aim target.
   */
  private boolean isAimedAtTarget() {
    boolean isRedAlliance =
        DriverStation.getAlliance().isPresent()
            && DriverStation.getAlliance().get() == Alliance.Red;
    Translation2d robotPosition = drive.getPose().getTranslation();
    Translation2d target;
    if (FieldConstants.isInOwnAllianceZone(robotPosition, isRedAlliance)) {
      target = FieldConstants.getHubCenter(isRedAlliance);
    } else {
      target = FieldConstants.getPassingTarget(robotPosition, isRedAlliance);
    }
    Translation2d robotToTarget = target.minus(robotPosition);
    double targetHeading =
        Math.atan2(robotToTarget.getY(), robotToTarget.getX())
            + Math.PI
            + Math.toRadians(ShooterConstants.shooterHeadingOffsetDegrees);
    double error = MathUtil.angleModulus(targetHeading - drive.getRotation().getRadians());
    boolean aimed = Math.abs(error) < kAimToleranceRad;
    Logger.recordOutput("Shooter/AimErrorDeg", Math.toDegrees(error));
    Logger.recordOutput("Shooter/AimedAtTarget", aimed);
    return aimed;
  }

  private Translation2d getLinearVelocityFromJoysticks() {
    double x = -drv.getLeftY();
    double y = -drv.getLeftX();
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), 0.1);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;
    return new Pose2d(Translation2d.kZero, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
        .getTranslation();
  }

  /** Rotate in place to aim back of robot at hub. */
  private Command aimAtHub() {
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

  /** Called at the start of teleop to reset subsystem states coming out of auto. */
  public void teleopInit() {
    shooter.setGoal(Shooter.Goal.IDLE);
    indexer.setGoal(Indexer.Goal.IDLE);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.selectedCommand();
  }

  /** Returns the AutoFactory for creating autonomous routines. */
  public AutoFactory getAutoFactory() {
    return autoFactory;
  }

  /** Returns the name of the currently selected autonomous routine for logging. */
  public String getSelectedAutoName() {
    return autoChooser.selectedCommand().getName();
  }
}
