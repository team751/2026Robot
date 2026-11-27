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
| `src/main/java/frc/robot/util/Dashboard.java` (new) | One place for all dashboard values. Replaces direct `SmartDashboard.putX` calls. Publishes to NetworkTables under the same `SmartDashboard/...` keys, so Elastic tile names do not change. Also has `putFieldRobotPose`, replacing `Field2d` + `putData`. |
| `Robot.java` | `robotInit()` is removed in 2027, so its body now runs at the end of the constructor, in the same order as before. |
| `Odometry.java`, `SwerveSubsystem.java`, `ControlBoard.java` | Live `SmartDashboard.put*` calls now go through `Dashboard`. `Odometry` no longer uses `Field2d`. Commented-out `SmartDashboard` lines were left alone. |
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
2. **Auto chooser (`RobotContainer.java`)**: convert to `Selectable`. Unverified: the exact
   Selectable API, and what PathPlanner's 2027 `AutoBuilder.buildAutoChooser()` returns. Also check
   whether AdvantageKit's 2027 `LoggedDashboardChooser` still wraps the old chooser.
3. **AdvantageKit**: confirm a build exists that pairs with WPILib alpha 7 (the newest AdvantageKit
   2027 release found while preparing this was alpha-6; alpha 6 and earlier vendordeps do not work
   with alpha 7). 2027 limitations to know about: alert logging disabled, `OpModeRobot` not
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

## Known 2027 changes not audited yet (the team said not to worry about them for now)

- `Rotation2d.getDegrees()/getRadians()/getRotations()` now return wrapped angles. Most uses here
  are logging, but any control math that needs continuous angles would change.
- Raw timestamps are nanoseconds, not microseconds (`RobotController.getTime`, NT timestamps).
  `Odometry.java` uses the CTRE clock (`Utils.getCurrentTimeSeconds()`), which is not affected by
  this, but this was not verified.
- "Test" mode is now "Utility" (no `testInit` etc. were found in the robot code).
- `MapSimSwerveTelemetry.java` uses `Mechanism2d`; `frc.lib.PS5Controller` wraps `Joystick`;
  `frc.lib.TunableParameter` has its `SmartDashboard` calls commented out. None were checked for
  2027 compatibility.
- Java 25 and Gradle 9.4.1 are required; Windows 10 is not supported by the 2027 tools.
