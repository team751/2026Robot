# WPILib 2027 upgrade framework (alpha 8, Systemcore only)

**Status: framework only. Nothing here has been compiled or tested against WPILib 2027.**
It was prepared without a Windows 11 machine, a 2027 toolchain, or access to the 2027 Maven
repositories. The team agreed it does not have to build yet. Every spot that is a guess is marked
`TODO(2027)` in the code. Search the project for `TODO(2027)` to find them all.

Decisions made with the team: target **alpha 8** (originally alpha 7; moved to alpha 8 on
2026-11-27, see "Why alpha 8" below; later alphas or Beta 1 are a separate step),
**Systemcore only** (no roboRIO), keep **PathPlanner** (it has a 2027 branch), use **Selectable**
for the auto chooser, keep the dashboard tiles **and** AdvantageKit logging, upgrade only (no
Commands v3 or other new features), and port the alliance-flip logic exactly as it is.

## Why alpha 8

WPILib's alpha 8 release notes (2026-10-30) describe it as quality-of-life fixes on top of alpha 7,
"largely compatible with Alpha 7 vendordeps". The main exception is the **C++** version of
PathPlanner, which doesn't affect us because we use Java. Every alpha-specific fact in this
framework was checked again at the `v2027.0.0-alpha-8` tag and still holds:
- `SmartDashboard` and `SendableChooser` are still gone, and `Field2d` still exists.
- `robotInit()` is still gone, and `utilityInit()` replaces `testInit()`.
- `CANPort`, `MatchState` and `RobotState` exist.
- The `/Telemetry` default prefix is the same.
- `TelemetryRegistry`, `NetworkTablesTelemetryBackend` and `PathUtil` are byte-identical to alpha 7.

Changes in alpha 8 that touch us:
- **Selectable backwards-compatibility shim** (allwpilib#9581): 2026-era dashboards that expect
  `SendableChooser`-style entries, such as Elastic, can show a `Selectable`. WPILib plans to
  **remove it before 2027 kickoff**, so don't depend on it long term.
- **DataLog:** match logs are no longer deleted, a USB stick mounted late gets the log transferred
  automatically, and low free space is reported as an alert. This is WPILib's `DataLogManager`;
  AdvantageKit's `WPILOGWriter` is separate (see item 4 below).
- **Kraken X60 `DCMotor` factories** use new CTRE test-lab values (allwpilib#9378). The sim
  (`SwerveSubsystem`, `DCMotor.getKrakenX60Foc`) will behave slightly differently. This doesn't
  change any code.
- **WPILib `Talon` and `VictorSP` removed:** not used here, since all motors are CTRE `TalonFX`.

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

1. Install WPILib **2027 alpha 8**. Running on a robot needs Systemcore image 14 or later and FIRST
   Driver Station alpha 7 or later.
2. In the 2027 VS Code, run **WPILib: Import a WPILib Project** on a copy of this branch. This is a
   2026 project, so it needs the full import. The in-place "update to alpha 8" prompt only applies to
   projects that are already alpha 7. The new project name must differ from the old folder, and the
   project must not be on OneDrive. Choose Systemcore as the platform.
3. Re-import every vendor library: AdvantageKit, CTRE Phoenix 6, PathPlanner (2027 branch), REVLib,
   PhotonLib, maple-sim, YALL. Alpha 6 and earlier vendordeps do not work. Alpha 7 vendordeps
   should work on alpha 8, except C++ PathPlanner. As of 2026-11-27, AdvantageKit's newest tag is
   `v27.0.0-alpha-7`, with no alpha 8 tag yet. **Stop and report any library with no alpha 7 or
   alpha 8 build**, and the team will review it in person.
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
   **The NT keys change by default.** At alpha 8, `RobotBase` (`org/wpilib/framework/RobotBase.java`
   line 227; line 224 at alpha 7) registers `TelemetryRegistry.registerBackend("", new NetworkTablesTelemetryBackend(inst,
   "/Telemetry"))`. So `Telemetry.log("Swerve/Pose x", ...)` publishes to `/Telemetry/Swerve/Pose x`,
   not `/SmartDashboard/Swerve/Pose x`, and every Elastic tile goes blank. Pick one:
   - **(a) Re-point the Elastic layout** to `/Telemetry/...`. This is the WPILib default, with no extra
     code to maintain.
   - **(b) Keep the old keys.** In the `Robot()` constructor, register
     `TelemetryRegistry.registerBackend("/SmartDashboard", new NetworkTablesTelemetryBackend(NetworkTableInstance.getDefault(), ""))`
     and have `Dashboard` log to `"SmartDashboard/" + key`. The registry picks the backend by longest
     prefix match and passes it the full normalized path. This backend adds no prefix, so the key
     comes out as `/SmartDashboard/<key>`. This was worked out from the source of
     `TelemetryRegistry`, `NetworkTablesTelemetryBackend` and `PathUtil`, which are identical at
     alpha 7 and alpha 8, and has **not been run**.
     Check the real keys in OutlineViewer or AdvantageScope before relying on it.
2. **Auto chooser (`RobotContainer.java`)**: convert to `Selectable`. The API is known.
   `org.wpilib.tunable.Selectable<V>` has `add`, `addDefault`, `remove`, `setDefault`, `clear`,
   `getSelected` and `onChange(Consumer<V>)`, and is published with
   `Tunables.publish("auto", chooser)` (`tunables/doc/tunables.md`, "Selectable\<V\>"). It is **not
   thread-safe**, so use it only from the main robot thread. The API is unchanged at alpha 8. Alpha
   8's shim lets the existing Elastic chooser tile show it for now, but the shim is planned to be
   removed before kickoff, so plan to use a 2027-aware dashboard widget. Still unverified: what PathPlanner's
   2027 `AutoBuilder.buildAutoChooser()` returns, and whether AdvantageKit's 2027
   `LoggedDashboardChooser` still wraps a chooser.
3. **AdvantageKit**: the newest build is `v27.0.0-alpha-7` (2026-10-23), and there is no alpha 8
   tag yet. WPILib says alpha 8 is largely compatible with alpha 7 vendordeps, so it should work,
   but that is **not confirmed** for AdvantageKit specifically. Alpha 6 and earlier do not work.
   2027 limitations to know about: alert logging disabled, `OpModeRobot` not
   supported (we use `LoggedRobot`, which is supported), console logging can lag, onboard IMU is not
   used in replay.
4. **Log location**: `Robot.java` still has roboRIO-era comments about a USB stick at `/U/logs`.
   `new WPILOGWriter()` with no arguments is kept (no hardcoded path). Check where it actually
   writes on Systemcore and update the comments. Alpha 8's automatic USB transfer and keeping match
   logs apply to WPILib's `DataLogManager`. Check whether AdvantageKit's writer gets the same
   behavior.
5. **PathPlanner and swerve**: confirm CTRE Phoenix 6 swerve and `TunerConstants` regeneration on
   Systemcore. CAN device classes now take a `CANPort` enum instead of an integer, which may affect
   the `CANBus` fields in `Robot.java` and `CTREConfig`.
6. **Alliance flip**: ported as is. Reminder that `DriverStation` is split into `MatchState` and
   `RobotState` in 2027; the importer attempts this, so review its output around
   `DriverStation.getAlliance()` in `Robot.java`, `SwerveSubsystem.java`, `HubShift.java` and
   `FieldConstants.java`.
7. **Kinematics renames (since alpha 7, same at alpha 8)**: "speeds" became "velocities" in
   wpimath. It is not known whether the importer renames these, so expect compile errors here:
   - `ChassisSpeeds` → `ChassisVelocities`, with fields `vx`/`vy`/`omega` instead of
     `vxMetersPerSecond`/`vyMetersPerSecond`/`omegaRadiansPerSecond`
   - `SwerveModuleState` → `SwerveModuleVelocity`
   - `toChassisSpeeds` → `toChassisVelocities`
   - `toSwerveModuleStates` → `toSwerveModuleVelocities`
   - `desaturateWheelSpeeds` → `desaturateWheelVelocities`

   Affected code:
   - `SwerveSubsystem.java`: the import, `getRobotRelativeSpeeds()`, `drive(...)`,
     `getChassisSpeeds()`, and the `vxMetersPerSecond`/`vyMetersPerSecond` use around line 282
   - `MapSimSwerveTelemetry.java`: the struct publishers and `ModuleStates[i].speedMetersPerSecond`
   - `MapleSimSwerveDrivetrain.java` around line 159 (`omegaRadiansPerSecond`)

   CTRE's `getState().ModuleStates`/`Speeds` types and PathPlanner's `AutoBuilder` speeds supplier
   follow whatever their 2027 builds use. Match those rather than guessing.

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
- Java 25 is required (not checked here). Gradle 9.4.1 is what allwpilib's own wrapper uses at
  alpha 8; robot projects probably match, but that was not confirmed. Windows 10 is not supported by
  the 2027 tools (64-bit Windows 11, Ubuntu 26.04 or macOS 15+).
