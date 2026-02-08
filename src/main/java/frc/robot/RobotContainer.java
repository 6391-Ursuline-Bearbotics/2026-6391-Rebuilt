// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeDeployIO;
import frc.robot.subsystems.intake.IntakeDeployIOSim;
import frc.robot.subsystems.intake.IntakeDeployIOTalonFX;
import frc.robot.subsystems.intake.IntakeRollerIO;
import frc.robot.subsystems.intake.IntakeRollerIOSim;
import frc.robot.subsystems.intake.IntakeRollerIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.vision.VisionIOQuestNav;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

  // Controllers
  private final CommandXboxController drv = new CommandXboxController(0);
  private final CommandXboxController op = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  // Current drive mode
  private DriveMode currentDriveMode = DriveMode.STANDARD;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
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

        // Vision with 2 Limelights + QuestNav
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOLimelight(camera0Name, drive::getRotation),
                new VisionIOLimelight(camera1Name, drive::getRotation),
                new VisionIOQuestNav("questnav"));

        // Intake with TalonFX hardware
        intake = new Intake(new IntakeDeployIOTalonFX(), new IntakeRollerIOTalonFX());

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
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        intake = new Intake(new IntakeDeployIOSim(), new IntakeRollerIOSim());
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
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIO() {},
                new VisionIO() {},
                new VisionIO() {});
        intake = new Intake(new IntakeDeployIO() {}, new IntakeRollerIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

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
                }));

    // Switch to X pattern when X button is pressed
    drv.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

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
  }

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
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isFlipped ? drive.getRotation().plus(new Rotation2d(Math.PI)) : drive.getRotation()));
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

    // Calculate angle to target
    Translation2d robotToTarget = target.minus(robotPosition);
    double targetHeading = Math.atan2(robotToTarget.getY(), robotToTarget.getX());

    double omega = aimTargetController.calculate(drive.getRotation().getRadians(), targetHeading);

    ChassisSpeeds speeds =
        new ChassisSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            omega);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            speeds,
            isRedAlliance
                ? drive.getRotation().plus(new Rotation2d(Math.PI))
                : drive.getRotation()));
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

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
