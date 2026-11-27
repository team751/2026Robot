# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build Commands

```bash
./gradlew build              # Full build (compile + tests)
./gradlew deploy             # Deploy to RoboRIO (must be connected via USB/Wi-Fi/Ethernet)
./gradlew simulateJava       # Run simulation with MapleSim physics + Sim GUI
./gradlew test               # Run JUnit 5 tests (src/test/java exists but is empty)
./gradlew test --tests 'frc.robot.SomeTest'   # Run a single test class
./gradlew spotlessApply      # Auto-format code (googleJavaFormat; CI has a spotless.yml workflow)
./gradlew replayWatch        # AdvantageKit log replay (custom JavaExec task in build.gradle)
```

CI (`.github/workflows/build.yml`) runs `./gradlew build` on Java 17 for every push/PR.

### Branch status: `wpilib-2027-upgrade`

This branch holds an **uncompiled, untested** WPILib 2027 alpha 7 framework (Systemcore only). `build.gradle` still targets 2026.2.1 / Java 17 on purpose — the 2027 importer regenerates it. Read `UPGRADE_2027.md` before touching build files or WPILib imports, and `grep -rn "TODO(2027)" src` for guesses. Don't "fix" `edu.wpi.first` imports or vendordeps by hand.

## Architecture Overview

FRC Team 751's 2026 REBUILT season robot. Java 17, WPILib 2026.2.1, CTRE Phoenix 6 swerve drive, AdvantageKit logging (`Robot extends LoggedRobot`).

### System Diagram

```
Robot.java (LoggedRobot / AdvantageKit, 50Hz)
  ├─ CommandScheduler
  │    ├─ SwerveSubsystem (swerve drive, PathPlanner AutoBuilder)
  │    │    └─ [sim] MapleSimSwerveDrivetrain (200Hz physics)
  │    ├─ Odometry (vision + wheel fusion, created in Robot() constructor)
  │    │    └─ LimelightSubsystem (dual AprilTag cameras)
  │    ├─ Superstructure (state machine coordinator)
  │    │    ├─ ShooterSubsystem
  │    │    ├─ IntakeSubsystem
  │    │    ├─ ExtenderSubsystem
  │    │    └─ TransferSubsystem
  ├─ RobotContainer (named commands + auto chooser)
  └─ ControlBoard (PS5 controller bindings)
```

### Initialization Order (Matters!)

1. `Robot()` constructor start — AdvantageKit `Logger` setup (**must precede any subsystem construction**). On a real robot it logs to a USB stick (`/U/logs`) — never the RIO flash; no stick = no logs.
2. `Robot()` constructor — creates `SwerveSubsystem`, `Odometry` (moved up from `teleopInit()` so vision logs from boot), and `ControlBoard` (wrapped in try/catch)
3. End of the `Robot()` constructor (was `robotInit()` before WPILib 2027, which removed it) — creates `RobotContainer` (named commands + auto chooser), `LimelightSubsystem.initLimsplz()`
4. `driverStationConnected()` — calls `ControlBoard.tryInit()`
5. `teleopInit()` — sets operator perspective based on alliance color

`autonomousInit()` sets `ShooterSubsystem.isAuto = true`.

### Periodic Loop (50Hz)

```
robotPeriodic():
  1. CommandScheduler.run()  ← all subsystem periodic() + commands
  2. ControlBoard.displayUI()
```

*Note: `TunableParameter.updateAll()` exists but is currently commented out.*

### Dashboard (`util/Dashboard.java`)

All dashboard values go through `Dashboard` (never call `SmartDashboard.put*` directly — 2027 removes it). For now it publishes under the same `SmartDashboard/...` NT keys so Elastic tiles keep working. **After the swap to the 2027 Telemetry API the default keys move to `/Telemetry/...`** (RobotBase's default backend prefix), which breaks the Elastic layout unless the layout is re-pointed or a `/SmartDashboard` backend is registered. See the `Dashboard` Javadoc for both options. The maple-sim source copy (`org.ironmaple`) also goes through `Dashboard` and calls it from the 200Hz sim `Notifier` thread, so `Dashboard` must stay thread-safe. `Odometry` keeps a `Field2d` and publishes it through `Dashboard.putField(key, field)`; that is the only place `SmartDashboard` is still called directly (it becomes `Telemetry.log(key, field)` in 2027).

## Key Design Patterns

### Singleton Subsystems

**All subsystems** use `getInstance()`. Never use `new` directly.

```java
private static MySubsystem instance;
public static MySubsystem getInstance() {
    if (instance == null) instance = new MySubsystem();
    return instance;
}
private MySubsystem() { }
```

### Request-Based State Machines

Subsystems decouple intent from execution. External code calls `request*()` methods; transitions happen atomically in `periodic()`.

```java
subsystem.requestIntaking();  // sets a flag
// In periodic(): switch(state) checks flags, transitions, clears flags
```

Not all subsystems implement this identically: IntakeSubsystem uses request flags; ExtenderSubsystem, ShooterSubsystem, and TransferSubsystem assign state directly in `request*()` methods (no deferred flag). Both patterns are present in the codebase.

### Superstructure Coordinator

`Superstructure.java` holds references to all mechanism subsystems and has a minimal PRE_HOME/IDLE state machine that currently does nothing (stub). Actual mechanism coordination is done directly via StartEndCommands in `ControlBoard.java`, not through Superstructure.

## CAN Bus Architecture

Three CAN buses (defined as static fields in `Robot.java`):

| Bus | Constant | Devices |
|-----|----------|---------|
| `drivebus` | `Robot.drivebus` | All swerve modules (drive/steer/CANcoder), Pigeon 2 IMU |
| `gamepiecebus` | `Robot.gamepiecebus` | Shooter, intake, extender, transfer motors |
| `rio` | `Robot.riobus` | Nothing (was climber, now removed) |

CAN IDs are defined in `frc.robot.util.Constants` (not `frc.robot.Constants`, which is an empty WPILib stub). The ID scheme: swerve is 10–40 by corner (FL/FR/BL/BR); all other motors are numbered by proximity to the nearest swerve corner and height on the robot.

## Subsystems

### Swerve Drive (`subsystems/drive/`)

CTRE generated swerve (`TunerSwerveDrivetrain`) with Kraken X60 FOC motors. Max speed 4.73 m/s.

- `generated/TunerConstants.java` — **Generated by Tuner X, DO NOT EDIT**. `OldTunerConstants.java` is a backup.
- `SwerveSubsystem.java` — Extends TunerSwerveDrivetrain, configures PathPlanner AutoBuilder
- `SwerveConstants.java` — Derived constants + PathPlanner RobotConfig
- `Odometry.java` — Fuses vision translation (not heading) with wheel odometry

### Vision (`subsystems/vision/`)

Dual Limelights (`LimelightSubsystem`; `PhotonvisionConstants.java` exists for a planned PhotonVision cutover, not yet wired in): `limelight-front` (3G, 10.7.51.71) and `limelight-side` (2, 10.7.51.75). Only translation from vision is used — heading always comes from Pigeon 2 gyro. `hasTarget()` requires both cameras to have targets.

### Intake (`subsystems/intake/`)

- **IntakeSubsystem**: Single TalonFX, states: IDLE/INTAKING/SPITTING. VoltageOut control.
- **ExtenderSubsystem**: Single TalonFX + 4 limit switches (DIO 2–5: frontLeft, backLeft, frontRight, backRight). States: IDLE/EXTENDING/EXTENDED/RETRACTING/RETRACTED/JIGGLE_IN. Extended when either back sensor triggers; retracted when either front sensor triggers.
- Constants in `IntakeConstants.java` (speeds, motor configs for both).

### Climber — REMOVED

`subsystems/climber/` was deleted (commit b2ba3fd). Commented-out references remain in `ControlBoard`, `Superstructure`, `Robot.java`, and the CAN IDs in `Constants.java`; recover the old code from git history if it's ever needed. `Robot.riobus` is now unused.

### Shooter (`subsystems/shooter/`)

Three TalonFX motors: flywheel (main), follow (opposed follower), and a built-in transfer motor. States: IDLE/SLOWSPIN/REVERSE/SHOOT/AASHOOT. Distance-based speed via `calculateShooterSpeed()` (linear curve, 50–600 cm range). `AASHOOT` fires at fixed 35 RPS without distance calculation — used by operator's `ShootCommand(noDistance=true)`. `requestSpit()` reverses only the transfer motor.

### Transfer (`subsystems/transfer/`)

Dual TalonFX motors (top/bottom). States: IDLE/TRANSFER/REVERSE. VoltageOut control. Note: ShooterSubsystem also has its own internal transfer motor — these are separate mechanisms.

## Operator Controls (ControlBoard.java)

### Driver (Port 0, PS5)

| Input | Action |
|-------|--------|
| Left stick | Translation |
| Right stick X | Rotation (squared for sensitivity) |
| Right bumper (hold) | Precise mode: 20% translation, 30% rotation |
| Left trigger (hold) | IntakeCommand |
| Left bumper (hold) | SpitCommand (transfer reverse + intake spit + extend extender) |
| D-pad up (hold) | Extend extender |
| D-pad down (hold) | Retract extender + run intake |
| Cross (hold) | JiggleCommand (unstick game pieces) |
| Right trigger (hold) | ShootCommand + auto-aim toward alliance hub + JiggleCommand simultaneously |
| D-pad left (hold) | SimpleUnstuck (briefly reverses transfer, restores prior state on release) |
| Right/Left stick press (hold) | Axis align to nearest trench |
| Circle | Reset rotation to alliance perspective |

### Operator (Port 1, PS5)

| Input | Action |
|-------|--------|
| Right trigger (hold) | ShootCommand fixed-speed (noDistance=true, AASHOOT mode) |
| Left trigger (hold) | IntakeCommand |
| D-pad up (hold) | Extend extender |
| D-pad down (hold) | Retract extender |
| D-pad left (hold) | Decrement aim angle offset by 5° |
| D-pad right (hold) | Increment aim angle offset by 5° |
| Cross (hold) | JiggleCommand |

*Note: Climber bindings are commented out. Square button spit binding is commented out on both controllers.*

### Drive Request Config

Field-centric with 5% translation deadband, 10% rotation deadband, OpenLoopVoltage drive, Position steer, DesaturateWheelSpeeds, OperatorPerspective.

## Hub Shift Tracking (`util/HubShift.java`)

Tracks whether our alliance's hub is active during teleop. It uses FMS game data (`'R'`/`'B'` names the alliance that goes inactive first) and match time for the TRANSITION → SHIFTS 1–4 → END GAME schedule. It only works with a real countdown (FMS or DS practice mode) and once game data has arrived; `isKnown()` reports whether both are true. `ControlBoard.displayUI()` publishes the results under `Hub/*`.

## Field Constants (`FieldConstants.java`)

Uses `GameElement` enum for all field elements (hubs, towers, trenches, outposts) with alliance-aware helpers:
- `getAllianceHub()` — returns hub Pose2d for current alliance
- `getNearestTrench(robotPose)` — returns closest alliance trench
- `GameElement.getColor(isBlue)` / `GameElement.getByType(type)` — filter elements

WPILib Blue-origin coordinates: origin at bottom-right of Blue wall, +X toward Red, +Y left.

## Autonomous

PathPlanner-based. AutoBuilder configured in `SwerveSubsystem` constructor. `RobotContainer.java` registers named commands for use in PathPlanner autos: `Shoot`/`StopShoot`, `Intake`/`StopIntake`, `Spit`/`StopSpit`, `Retract`, `Transfer`/`StopTransfer`. Auto chooser built in `RobotContainer` and scans `src/main/deploy/pathplanner/autos/*.auto` — no code changes needed for new autos.

### Mirroring Autos

To mirror an auto for the opposite side of the field, create **new** path files (never edit the originals) and a new `.auto` file referencing them. Always make sure that reset odometry is turned off for all autos unless you are specifically told to leave it on.

**Field dimensions:**
- Field length (X): `16.540988` m (from `FieldConstants.FIELD_LENGTH`)
- Field width (Y): `8.052` m

**Flip formulas:**

| Mirror type | X | Y | Rotation |
|---|---|---|---|
| Blue ↔ Red (X-flip) | `16.540988 - oldX` | unchanged | `180 - oldRotation` |
| Top ↔ Bottom (Y-flip) | unchanged | `8.052 - oldY` | `-oldRotation` |
| Both (X+Y flip) | `16.540988 - oldX` | `8.052 - oldY` | `oldRotation - 180` |

**What to flip in each `.path` file:**
- All `anchor`, `prevControl`, and `nextControl` x/y coordinates
- All `rotationDegrees` in `rotationTargets`
- `goalEndState.rotation`
- `idealStartingState.rotation`

**Steps:**
1. Read each source `.path` file
2. Apply the appropriate flip formula to all coordinates and rotations
3. Write new `.path` files with a descriptive suffix (e.g., `red`, `bottom`, `red bottom`)
4. Create a new `.auto` file with the same command structure, referencing the new path names

**Naming conventions used:**
- X-flip (blue→red): add `red` suffix (e.g., `depot red`, `intakedepot red`)
- Y-flip (top→bottom): add `bottom` suffix (e.g., `depot bottom`, `intakedepot bottom`)
- X+Y flip: add `red bottom` suffix (e.g., `depot red bottom`, `intakedepot red bottom`)
- Auto files: `Blueleftdepot-top.auto` → `Redleftdepot-bottom.auto`, etc.

**Notes:**
- Bezier control points can go outside field bounds (negative values are valid)
- Constraint zones, event markers, and global constraints are copied unchanged
- Velocities are copied unchanged — only positions and rotations are flipped
- Normalize rotation values by adding/subtracting 360° if they fall outside a readable range
- Ensure that reset odometry isn't turned on. If it is, turn it off.

## Commands (`commands/`)

- **IntakeCommand** — coordinates intake sequence
- **ShootCommand** — runs ShooterSubsystem + TransferSubsystem together; accepts `noDistance` boolean (true = AASHOOT fixed-speed mode); both go idle on end
- **JiggleCommand** — oscillates extender in/out while coordinating intake/transfer/shooter; used to unstick game pieces
- **SpitCommand** — reverses transfer + spits intake + extends extender; all mechanisms idle/retract on end
- **SimpleUnstuck** — reverses transfer while held, restores the prior `TransferState` on release
- **WiggleCommand** — empty stub, not yet implemented

## Simulation

IronMaple library (`org.ironmaple.simulation`) + Dyn4j physics at 200Hz (5ms loop via `Notifier`). Includes full arena simulation with game pieces (`RebuiltFuelOnField`), `SwerveDriveSimulation`, and `IntakeSimulation`. Run with `./gradlew simulateJava`. Connect AdvantageScope to `localhost` for 3D viz. Vision returns null in sim.

## Utility Classes (`frc.lib`)

- **CTREConfig** — Builder for CTRE devices: `.withName()`, `.withCanID()`, `.withBus()`, `.createDevice(TalonFX::new)`
- **CTREUtil** — `tryUntilOK()` retries CTRE calls up to 10x. `applyConfiguration()` for TalonFX/CANcoder/etc.
- **PS5Controller** — WPILib Joystick wrapper. All axes inverted (up/left = positive). Fields: `leftVerticalJoystick`, `rightHorizontalJoystick`, `leftBumper`, `rightTrigger`, `crossButton`, etc.
- **TunableParameter** — Live SmartDashboard tuning: `new TunableParameter("key", default, callback)`. Polled in `robotPeriodic()`.

## Adding a New Subsystem

1. Create `subsystems/mysubsystem/MySubsystem.java` with singleton pattern
2. Create `MySubsystemConstants.java` with `CTREConfig` motor configs
3. Add CAN IDs to `frc.robot.util.Constants`
4. Add reference in `Superstructure.java`
5. Initialize in `Robot.java` if needed outside Superstructure
6. Add controller bindings in `ControlBoard.java`
7. Follow request-based state machine pattern

## Detailed Documentation

See `docs/` for comprehensive reference:
- `docs/architecture.md` — System design, lifecycle, data flow
- `docs/swerve-drive.md` — Hardware, PID tuning, CTRE API
- `docs/vision-and-odometry.md` — Limelight config, sensor fusion
- `docs/autonomous.md` — PathPlanner, creating autos
- `docs/simulation.md` — MapleSim, AdvantageScope
- `docs/subsystems.md` — Subsystem reference, templates
- `docs/field-constants.md` — FieldConstants and GameElement reference
- `docs/utilities.md` — CTREConfig, CTREUtil, TunableParameter, PS5Controller reference
- `docs/getting-started.md` — Setup for new contributors
- `docs/external-resources.md` — All vendor docs and links
