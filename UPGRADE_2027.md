# WPILib 2027 upgrade framework (alpha 7, Systemcore only)

**Status: framework only. Nothing here has been compiled or tested against WPILib 2027.**
It was prepared without a Windows 11 machine, a 2027 toolchain, or access to the 2027 Maven
repositories. The team agreed it does not have to build yet. Every spot that is a guess is marked
`TODO(2027)` in the code. Search the project for `TODO(2027)` to find them all.

Decisions made with the team: target **alpha 7 only** (move to later alphas as a separate step),
**Systemcore only** (no roboRIO), keep **PathPlanner** (it has a 2027 branch), use **Selectable**
for the auto chooser, keep the dashboard tiles **and** AdvantageKit logging, upgrade only (no
Commands v3 or other new features), and port the alliance-flip logic exactly as it is.

## What was changed in this branch

| File | Change |
|---|---|
| `src/main/java/frc/robot/util/Dashboard.java` (new) | One place for all dashboard values. Replaces direct `SmartDashboard.putX` calls. For now it publishes to NetworkTables under the same `SmartDashboard/...` keys, so Elastic tile names do not change **yet**. The default keys change after the Telemetry swap (see "Still to do" item 1). `putField(key, Field2d)` wraps `SmartDashboard.putData` (the only direct SmartDashboard call left); it becomes `Telemetry.log(key, field)` in 2027. Publisher caches are `ConcurrentHashMap`s because the sim thread calls it too. |
| `org/ironmaple/simulation/SimulatedArena.java`, `.../motorsims/SimulatedBattery.java` | The copied maple-sim source called `SmartDashboard.putNumber` directly, which won't compile on 2027 (`SmartDashboard` is gone). These now go through `Dashboard`, with the same keys. They run on the 200Hz sim `Notifier` thread. |
| `Robot.java` | `robotInit()` is removed in 2027, so its body now runs at the end of the constructor, in the same order as before. |
| `Odometry.java`, `SwerveSubsystem.java`, `ControlBoard.java` | Live `SmartDashboard.put*` calls now go through `Dashboard`. `Odometry` keeps its `Field2d` and publishes it with `Dashboard.putField`. Commented-out `SmartDashboard` lines were left alone. |
| `RobotContainer.java` | TODO(2027) comment only. The chooser is **not converted to Selectable yet** (see below). |
| `.claude/CLAUDE.md` | Initialization order updated for the missing `robotInit()`. |

Not touched on purpose: package names (`edu.wpi.first` becomes `org.wpilib`), `build.gradle`,
`vendordeps/`, `.wpilib/`, and the Gradle wrapper. The 2027 importer does the package renames and
regenerates the Gradle files, and guessing those by hand would put wrong names in the code.

## Steps for whoever has a Windows 11 (or macOS 15 / Ubuntu 26.04) machine

1. Install WPILib **2027 alpha 7** (Systemcore image 14 and the alpha 7 Driver Station are needed to
   run on a robot).
2. In the 2027 VS Code, run **WPILib: Import a WPILib Project** on a copy of this branch. In-place
   upgrades are not supported, the new project name must differ from the old folder, and the project
   must not be on OneDrive. Choose Systemcore as the platform.
3. Re-import every vendor library for alpha 7 (alpha 6 and earlier vendordeps do not work):
   AdvantageKit, CTRE Phoenix 6, PathPlanner (2027 branch), REVLib, PhotonLib, maple-sim, YALL.
   **Stop and report any library with no alpha 7 build**; the team will review it in person.
4. The importer regenerates `build.gradle` and drops custom edits. **Re-apply these** (they are in
   the 2026 `build.gradle` in this branch):
   - The AdvantageKit annotation processor: read `vendordeps/AdvantageKit.json` with `JsonSlurper`
     and add `annotationProcessor "org.littletonrobotics.akit:akit-autolog:$akitJson.version"`.
   - The `replayWatch` `JavaExec` task (`org.littletonrobotics.junction.ReplayWatch`).
   - The `spotless` block (googleJavaFormat, `enforceCheck false`).
5. Run `./gradlew build`, then work through the errors with the list below.

## Still to do (all marked `TODO(2027)` in the code)

1. **Dashboard.java**: swap the NetworkTables bodies for the Telemetry API. The migration guides
   are the "Migration from WPILib 2026" sections of `telemetry/doc/telemetry.md` and
   `tunables/doc/tunables.md` in wpilibsuite/allwpilib. Keep the method signatures so no
   subsystem has to change.
   **The NT keys change by default.** At alpha 7, `RobotBase` (`org/wpilib/framework/RobotBase.java`
   line 224) registers `TelemetryRegistry.registerBackend("", new NetworkTablesTelemetryBackend(inst,
   "/Telemetry"))`. So `Telemetry.log("Swerve/Pose x", ...)` publishes to `/Telemetry/Swerve/Pose x`,
   not `/SmartDashboard/Swerve/Pose x`, and every Elastic tile goes blank. Pick one:
   - **(a) Re-point the Elastic layout** to `/Telemetry/...`. This is the WPILib default, with no extra
     code to maintain.
   - **(b) Keep the old keys.** In the `Robot()` constructor, register
     `TelemetryRegistry.registerBackend("/SmartDashboard", new NetworkTablesTelemetryBackend(NetworkTableInstance.getDefault(), ""))`
     and have `Dashboard` log to `"SmartDashboard/" + key`. The registry picks the backend by longest
     prefix match and passes it the full normalized path. This backend adds no prefix, so the key
     comes out as `/SmartDashboard/<key>`. This was worked out from the alpha-7 source
     (`TelemetryRegistry`, `NetworkTablesTelemetryBackend`, `PathUtil`) and has **not been run**.
     Check the real keys in OutlineViewer or AdvantageScope before relying on it.
2. **Auto chooser (`RobotContainer.java`)**: convert to `Selectable`. The API is known.
   `org.wpilib.tunable.Selectable<V>` has `add`, `addDefault`, `remove`, `setDefault`, `clear`,
   `getSelected` and `onChange(Consumer<V>)`, and is published with
   `Tunables.publish("auto", chooser)` (`tunables/doc/tunables.md`, "Selectable\<V\>"). It is **not
   thread-safe**, so use it only from the main robot thread. Still unverified: what PathPlanner's
   2027 `AutoBuilder.buildAutoChooser()` returns, and whether AdvantageKit's 2027
   `LoggedDashboardChooser` still wraps a chooser.
3. **AdvantageKit**: a `v27.0.0-alpha-7` tag exists in `Mechanical-Advantage/AdvantageKit`, but its
   release notes were not checked, so it is **not confirmed** that it pairs with WPILib alpha 7
   (alpha 6 and earlier vendordeps do not work with alpha 7). 2027 limitations to know about: alert
   logging disabled, `OpModeRobot` not
   supported (we use `LoggedRobot`, which is supported), console logging can lag, onboard IMU is not
   used in replay.
4. **Log location**: `Robot.java` still has roboRIO-era comments about a USB stick at `/U/logs`.
   `new WPILOGWriter()` with no arguments is kept (no hardcoded path). Check where it actually
   writes on Systemcore and update the comments.
5. **PathPlanner and swerve**: confirm CTRE Phoenix 6 swerve and `TunerConstants` regeneration on
   Systemcore. CAN device classes now take a `CANPort` enum instead of an integer, which may affect
   the `CANBus` fields in `Robot.java` and `CTREConfig`.
6. **Alliance flip**: ported as is. Reminder that `DriverStation` is split into `MatchState` and
   `RobotState` in 2027; the importer attempts this, so review its output around
   `DriverStation.getAlliance()` in `Robot.java`, `SwerveSubsystem.java`, `HubShift.java` and
   `FieldConstants.java`.

## Audited: no code change needed

- **Wrapped angles.** `Rotation2d.getDegrees()/getRadians()/getRotations()` now return wrapped
  angles (−180…180°). Every use outside logging was checked:
  - **Auto-aim and axis-align** (`ControlBoard`) both feed `autoAimController`, which has
    `enableContinuousInput(-180, 180)`, so wrapping makes no difference. `getAxisAlignAngle` also
    wraps its input itself.
  - **`FieldConstants.calculateAngleDifference`** goes through `minus()`, which was already wrapped
    in 2026 because `rotateBy` rebuilds the angle from cos/sin.
  - **`FieldConstants` offset math** only uses cos/sin, which ignore wrapping.
  - **`SwerveSubsystem.setOperatorPerspectiveAndAdjustPose`** compares two values that both go
    through the same wrapping, and turns the delta back into a `Rotation2d`.
  - **Limelight yaw input** (`SetRobotOrientation`) accepts any angle.

## Known 2027 changes not audited yet (the team said not to worry about them for now)

- Raw timestamps are nanoseconds, not microseconds (`RobotController.getTime`, NT timestamps).
  `Odometry.java` uses the CTRE clock (`Utils.getCurrentTimeSeconds()`), which is not affected by
  this, but this was not verified.
- "Test" mode is now "Utility" (no `testInit` etc. were found in the robot code).
- `MapSimSwerveTelemetry.java` uses `Mechanism2d`; `frc.lib.PS5Controller` wraps `Joystick`;
  `frc.lib.TunableParameter` has its `SmartDashboard` calls commented out. None were checked for
  2027 compatibility.
- Java 25 and Gradle 9.4.1 are required; Windows 10 is not supported by the 2027 tools.
