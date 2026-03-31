# FRC Team 6391 — 2026 Robot Code (REBUILT)

Robot software for Team 6391's 2026 FRC competition robot, built on the
**[AdvantageKit TalonFX Swerve Template](https://github.com/Mechanical-Advantage/AdvantageKit)**
by Littleton Robotics (6328).

**Season results:** Rank 2 at St. Louis Regional (13-4-0) · Regional Finalists · Autonomous Award

---

## Table of Contents

- [Hardware Overview](#hardware-overview)
- [Software Stack](#software-stack)
- [Subsystems](#subsystems)
- [Autonomous Routines](#autonomous-routines)
- [Controller Bindings](#controller-bindings)
- [CAN IDs](#can-ids)
- [Build & Deploy](#build--deploy)
- [Simulation & Replay](#simulation--replay)
- [Log Analysis Tools](#log-analysis-tools)

---

## Hardware Overview

| System | Hardware |
|---|---|
| **Drive** | 4-module swerve, TalonFX (Kraken X60) drive + turn, CANcoder absolute encoders |
| **Gyro** | Pigeon 2.0 |
| **Shooter** | Dual TalonFX (Kraken X60) flywheel, adjustable servo hood |
| **Indexer** | TalonFX belt motor, TalonFX kicker motor, 2× NEO 550 / SparkMAX spinners |
| **Intake** | TalonFX deploy motor (25:1 reduction, current-spike hard stop), TalonFX roller (1:1) |
| **Vision** | 2× Limelight (AprilTag pose estimation), QuestNav (real); PhotonVision (sim) |
| **Controller** | NI roboRIO 2, REV PDH |

---

## Software Stack

| Library | Version | Purpose |
|---|---|---|
| WPILib | 2026 | Robot framework, command-based |
| AdvantageKit | 2026 | Deterministic replay logging (`@AutoLog`, IO layers) |
| Phoenix 6 | 26.1.2 | TalonFX / Pigeon2 motor control & odometry |
| ChoreoLib | 2026 | Trajectory following for autonomous |
| REVLib | — | SparkMAX (NEO 550 spinners) |
| PhotonVision | — | Simulation pose estimation |

**Build toolchain:** Gradle + WPILib JDK 17 at `C:\Users\Public\wpilib\2026\jdk`

---

## Subsystems

### Drive

*`src/main/java/frc/robot/subsystems/drive/`*

Four-module swerve drivetrain using the AdvantageKit `Module` / `ModuleIO` pattern.

- **Modules:** FL (11/12/13), FR (21/22/23), BR (31/32/33), BL (41/42/43) — CAN IDs are drive/turn/encoder
- **High-frequency odometry** via `PhoenixOdometryThread` — status signals at 250 Hz, odometry thread runs independently of the main loop
- **Field-relative drive** with heading lock and joystick deadband
- **Trajectory following** — `drive.followTrajectory(SwerveSample)` with combined feedforward + PID (kP = 5.0 for x/y/heading)
- **SysId** routines wired for characterizing drive and turn motors
- **Alliance flipping** — all poses and trajectories automatically mirror for Red alliance

### Shooter

*`src/main/java/frc/robot/subsystems/shooter/`*

Dual-flywheel shooter with distance-based RPM and hood angle interpolation.

- **Left motor** (CAN 7) and **Right motor** (CAN 8), both Kraken X60, stator limit 80A
- **Hood** — two servos (PWM 0/1) adjust launch angle between 20°–45°
- **Interpolation tables** in `ShooterConstants` map distance → RPM, distance → hood angle, and distance → time-of-flight (for shoot-on-move compensation)
  - Effective range: ~1.3 m (close shot) to ~6.5 m (long pass)
- **Shoot-on-move** — predicts ball landing position using robot velocity and TOF table; iterates 3× for accuracy
- **Hub targeting** — `Shooter.Goal.SHOOT` auto-tracks the hub using vision pose and field geometry; `shooterHeadingOffsetDegrees` in `ShooterConstants` trims physical misalignment
- **Goals:** `IDLE`, `SHOOT` (auto-aim), `SHOOT_STATIONARY`, `EJECT`, `PASS`

### Indexer

*`src/main/java/frc/robot/subsystems/indexer/`*

Three-motor game piece path from intake to shooter.

- **Belt** (CAN 5, TalonFX) — feeds game pieces up toward the kicker at 900 RPM
- **Kicker** (CAN 6, TalonFX) — final high-speed kick into shooter wheels at 4000 RPM
- **Spinners** (CAN 9/10, NEO 550 + SparkMAX) — lateral centering rollers; 12:60 reduction, duty-cycle control, stall detection with cooldown
- **Jam detection** — stator current threshold (40A) with debounce; auto-reverses for 0.25s on jam
- **Goals:** `IDLE`, `FEED`, `EJECT`

### Intake

*`src/main/java/frc/robot/subsystems/intake/`*

Over-the-bump ground intake with state-machine deploy.

- **Deploy motor** (CAN 4, TalonFX, 25:1) — open-loop voltage control (2.8V deploy / −2.8V retract); detects hard stops via stator current spike (30A deploy / 45A retract threshold)
- **Roller motor** (CAN 3, TalonFX, 1:1) — velocity-controlled at 3500 RPM (intake) / 4500 RPM (clump) / −4500 RPM (eject)
- **Deploy states:** `RETRACTED → DEPLOYING → DEPLOYED → RETRACTING` (state machine, not position control)
- **Roller jam detection** — stator threshold 40A with 0.5s debounce; auto-reverses 0.25s
- **Goals:** `IDLE`, `INTAKE`, `EJECT`, `DEPLOYED_IDLE`

### Vision

*`src/main/java/frc/robot/subsystems/vision/`*

Multi-camera AprilTag pose estimation fused into drive odometry.

- **Real:** 2× Limelight (`VisionIOLimelight`) + QuestNav (`VisionIOQuestNav`)
- **Sim:** PhotonVision (`VisionIOPhotonVisionSim`)
- Pose observations are filtered by tag count and ambiguity before being passed to the drive pose estimator via standard deviations

---

## Autonomous Routines

All autos use [Choreo](https://github.com/SleipnirGroup/Choreo) for trajectory following. Trajectories live in `src/main/deploy/choreo/`. Alliance flipping is handled automatically.

The auto selector runs on SmartDashboard / Elastic via `AutoChooser`.

| Routine | Description |
|---|---|
| **Depot Cycle** | Simple depot-side cycle trajectory |
| **Depot Inside** | Depot inside path variant |
| **Depot Single Pass** | Cross bump → intake single pass → return → shoot |
| **Depot Single Pass Shoot First** | Shoot preload → cross bump → intake → return → shoot |
| **Depot Single Pass Shoot On Move** | Cross bump → intake → return → shoot while driving toward depot at 0.5 m/s; deploys intake near depot to collect staged balls |
| **Depot Double Pass** | Cross bump → intake double pass → return → shoot |
| **Depot Double Pass Shoot First** | Shoot preload first, then double pass |
| **Outpost Single Pass** | Outpost-side single pass variant |
| **Outpost Single Pass Shoot First** | Outpost single pass with preload shot |
| **Outpost Double Pass** | Outpost-side double pass |
| **Outpost Double Pass Shoot First** | Outpost double pass with preload shot |
| **Outpost FULL Pass** | Outpost full-field pass trajectory |
| **Safe** | Short safe-zone trajectory + shoot |
| **Safe Shoot First** | Shoot preload → safe trajectory |
| **Shoot Only** | Aim at hub and shoot preloaded ball only |

**Common auto sequence pattern:**
1. Reset odometry from trajectory start pose
2. (Optional) Shoot preloaded ball
3. Deploy intake, cross bump via Choreo trajectory
4. Intake game pieces across the field
5. Spin up shooter, retract intake, return over bump
6. Sprint-to-pose correction after bump (PID, not trajectory)
7. Aim at hub and feed for remaining time

---

## Controller Bindings

### Driver (Port 0 — Xbox)

| Input | Action |
|---|---|
| Left stick | Translation (field-relative) |
| Right stick X | Rotation |
| Left trigger | Slow mode |
| Right trigger | Sprint |
| Start | Reset gyro heading |
| B | X-stance (defense lock) |
| Right bumper | Aim at hub (heading snap) |

### Operator (Port 1 — Xbox)

| Input | Action |
|---|---|
| A | Intake (deploy + roller) |
| B | Idle (retract intake, stop all) |
| Y | Deployed idle (intake deployed, roller off) |
| X | Eject |
| Right bumper | Feed (shoot when ready) |
| Left bumper | Shoot stationary |
| Right trigger | Shoot on move |
| Left trigger | Pass shot |
| D-pad Up | Manual hood up |
| D-pad Down | Manual hood down |

---

## CAN IDs

| Device | ID | Notes |
|---|---|---|
| Intake Roller | 3 | TalonFX |
| Intake Deploy | 4 | TalonFX, 25:1 |
| Indexer Belt | 5 | TalonFX |
| Indexer Kicker | 6 | TalonFX |
| Shooter Left | 7 | TalonFX (Kraken X60) |
| Shooter Right | 8 | TalonFX (Kraken X60) |
| Indexer Spinner Left | 9 | NEO 550 / SparkMAX |
| Indexer Spinner Right | 10 | NEO 550 / SparkMAX |
| Swerve FL Drive | 11 | TalonFX |
| Swerve FL Turn | 12 | TalonFX |
| Swerve FL Encoder | 13 | CANcoder |
| Swerve FR Drive | 21 | TalonFX |
| Swerve FR Turn | 22 | TalonFX |
| Swerve FR Encoder | 23 | CANcoder |
| Swerve BR Drive | 31 | TalonFX |
| Swerve BR Turn | 32 | TalonFX |
| Swerve BR Encoder | 33 | CANcoder |
| Swerve BL Drive | 41 | TalonFX |
| Swerve BL Turn | 42 | TalonFX |
| Swerve BL Encoder | 43 | CANcoder |
| Pigeon 2.0 | 1 | |

All devices on default CAN bus (`rio`).

---

## Build & Deploy

> **Requires WPILib JDK 17** at `C:\Users\Public\wpilib\2026\jdk`

```bash
# Build (runs Spotless formatter automatically — expect file changes)
./gradlew build -Dorg.gradle.java.home="C:\Users\Public\wpilib\2026\jdk"

# Deploy to robot
./gradlew deploy -Dorg.gradle.java.home="C:\Users\Public\wpilib\2026\jdk"

# Run tests
./gradlew test -Dorg.gradle.java.home="C:\Users\Public\wpilib\2026\jdk"
```

**Spotless** (Google Java Format) runs automatically before compile and reformats code in place. This is expected — commit the formatted output.

**Tuning mode:** Set `Constants.tuningMode = true` to expose PID gains as tunables via NetworkTables/AdvantageScope without redeploying.

---

## Simulation & Replay

Set `Constants.simMode` in `Constants.java`:

| Value | Behavior |
|---|---|
| `Mode.SIM` | Full physics simulation — `DCMotorSim` for all motors, PhotonVision for cameras |
| `Mode.REPLAY` | Replay from a `.wpilog` file — all IO interfaces return empty defaults, logged inputs are replayed deterministically |

To replay a log: set `simMode = Mode.REPLAY`, place the log file in the project, and configure the log path in `Robot.java`.

---

## Log Analysis Tools

Three Python scripts in `tools/` analyze AdvantageKit `.wpilog` files from competition.

**Requirement:** Python 3.10+ with `robotpy-wpiutil` (`pip install robotpy-wpiutil`)

**Run with `-X utf8` on Windows** to avoid encoding issues with special characters in output.

```bash
# All three tools accept the same base arguments:
python -X utf8 tools/<script>.py  <path/to/logs/>  [options]

# -o report.txt    Save output to a file (also prints to console)
# --no-practice    Skip practice match logs (p## files)
# --match "Qual 5" Analyze only one specific match
```

Log files follow the AdvantageKit naming convention:
`akit_<date>_<time>_<event>_<type><num>.wpilog` where type is `p`=practice, `q`=qual, `e`=elim.

---

### `tools/log_analyzer.py` — General Drivetrain Health

```bash
python -X utf8 tools/log_analyzer.py "C:\path\to\logs" --no-practice -o report.txt
```

Produces per-match tables for:
- Swerve drive current (mean, p95, peak) per module with inter-module spread flagging
- Drive + turn motor temperatures
- Wheel speed (mean & peak m/s) per module
- Acceleration/deceleration trends with bar chart
- Battery voltage (mean, min, max) — brownout risk flagging
- RoboRIO loop time — overrun detection
- **Anomaly summary** — all CRIT/WARN flags in one place at the bottom

---

### `tools/slip_analyzer.py` — Swerve Wheel Slip Detection

```bash
python -X utf8 tools/slip_analyzer.py "C:\path\to\logs" --module 3   # 0=FL 1=FR 2=BR 3=BL
```

Detects wheels that are spinning without gripping carpet by analyzing the
**current-per-rad/s ratio**. A healthy wheel scales current with velocity;
a slipping or damaged wheel spins fast for fewer amps.

Produces:
- CPR (current-per-rad/s) per module across all matches
- CPR ratio of the focus module vs average of other three — progressive drops indicate wear
- Velocity signature table: median current in each speed bin (0–80 rad/s)
- Pearson correlation between velocity and current
- Slip event rate (high velocity + low current samples)

*STL finding: BL degraded from ratio 1.03 (Practice 4) to 0.815 (Elim 6), consistent with progressive bevel gear slip.*

---

### `tools/mechanism_analyzer.py` — All Non-Drive Motors

```bash
python -X utf8 tools/mechanism_analyzer.py "C:\path\to\logs" --no-practice -o mechanisms.txt
```

Covers shooter, indexer, intake, PDH, and system alerts:
- **Shooter L/R** — current balance, CPR (which flywheel is free-spinning), velocity imbalance, temperatures, estimated shot count
- **Indexer belt/kicker** — current trends, jam events (stall = high current + low velocity)
- **Indexer spinners** — left/right current balance
- **Intake deploy** — hard-stop spike count (stator >30A = arm at physical stop)
- **Intake roller** — current, jam events, **motor disconnection count** per match
- **PDH** — total robot current and power
- **Active alerts** — all WPILib Alert messages fired during enabled periods

*STL finding: Intake roller disconnected 7× in Qual 5 and 3× in Qual 21 — highest priority for championship inspection.*
