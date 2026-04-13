# Autonomous

This document covers how autonomous routines work, how PathPlanner is integrated, and how to create new autonomous paths and routines.

## Overview

During the autonomous period (first portion of each match), the robot runs pre-programmed routines without driver input. This project uses [PathPlanner](https://pathplanner.dev/) for path creation and trajectory following.

### How It Works

1. **Before the match**: The drive team selects an autonomous routine from SmartDashboard's "Auto Chooser" dropdown
2. **Auto starts**: `Robot.autonomousInit()` schedules the selected command
3. **During auto**: PathPlanner follows the trajectory, calling `SwerveSubsystem.drive()` with calculated speeds
4. **Auto ends**: `Robot.autonomousExit()` cancels the auto command

## PathPlanner Integration

### AutoBuilder Configuration

PathPlanner's `AutoBuilder` is configured in the `SwerveSubsystem` constructor:

```java
AutoBuilder.configure(
    this::getPose,                     // How to get current robot pose
    this::resetPose,                   // How to reset odometry (for starting position)
    this::getRobotRelativeSpeeds,      // How to get current chassis speeds
    this::drive,                       // How to command the drivetrain
    new PPHolonomicDriveController(
        new PIDConstants(0.5, 0.0, 0.0),   // Translation PID (proportional only)
        new PIDConstants(0.3, 0.0, 0.0)    // Rotation PID (proportional only)
    ),
    SwerveConstants.robotConfig,       // Robot physical configuration
    () -> isRedAlliance(),             // Flip paths for red alliance
    this                               // Subsystem requirement
);
```

### Drive Method for Autonomous

PathPlanner calls the `drive()` method with robot-relative `ChassisSpeeds` and `DriveFeedforwards`:

```java
private void drive(ChassisSpeeds robotSpeeds, DriveFeedforwards feedForward) {
    this.setControl(m_pathApplyRobotSpeeds.withSpeeds(robotSpeeds));
}
```

The `ApplyRobotSpeeds` request uses **velocity drive** (closed-loop) and **position steer** (closed-loop) for maximum path accuracy.

### Auto Chooser

The auto chooser is built in `Robot.robotInit()`:

```java
autoChooser = AutoBuilder.buildAutoChooser();
SmartDashboard.putData("Auto Chooser", autoChooser);
```

This automatically scans `src/main/deploy/pathplanner/autos/` for all `.auto` files and creates a dropdown selector. No code changes are needed when adding new auto files.

### Alliance Flipping

PathPlanner automatically mirrors paths for the Red alliance. The `AutoBuilder.configure()` call includes a supplier that returns `true` when the robot is on Red alliance. This flips all coordinates so paths designed for Blue also work on Red.

## Robot Configuration

The PathPlanner robot configuration is defined in two places:

### In Code (SwerveConstants.java)

```java
// PathPlanner RobotConfig
ModuleConfig moduleConfig = new ModuleConfig(
    0.05,                           // Wheel radius (meters)
    10,                             // Max module speed (m/s)
    1.0,                            // Wheel COF
    DCMotor.getKrakenX60Foc(1),     // Drive motor model
    30400,                          // Drive motor max RPM
    1                               // Number of drive motors per module
);

Translation2d[] moduleOffsets = {
    new Translation2d(-0.263525, 0.263525),   // FL
    new Translation2d(0.263525, 0.263525),    // FR
    new Translation2d(-0.263525, -0.263525),  // BL
    new Translation2d(0.263525, -0.263525)    // BR
};

RobotConfig robotConfig = new RobotConfig(50, 5, moduleConfig, moduleOffsets);
// Mass: 50 kg, MOI: 5 kg*m^2
```

### In PathPlanner GUI (settings.json)

```json
{
    "robotWidth": 0.9,
    "robotLength": 0.9,
    "holonomicMode": true,
    "defaultMaxVel": 3.084,
    "defaultMaxAccel": 5.3,
    "defaultMaxAngVel": 540.0,
    "defaultMaxAngAccel": 720.0,
    "robotMass": 49.8,
    "robotMOI": 6.883,
    "driveMotorType": "krakenX60",
    "driveCurrentLimit": 60.0,
    "wheelCOF": 1.2,
    "maxDriveSpeed": 5.45
}
```

**Important**: Keep these two configurations in sync. The code configuration is used at runtime; the GUI configuration is used for path generation visualization.

## File Structure

```
src/main/deploy/pathplanner/
  settings.json     # Robot configuration for PathPlanner GUI
  navgrid.json      # Navigation grid for obstacle avoidance
  autos/            # Autonomous routines (.auto files)
    Forward.auto
    Auto1blue.auto
    Test Auto.auto
    red8path.auto
    redtestauto.auto
  paths/            # Individual path segments (.path files)
    Forward Path.path
    Park.path
    Score Intake.path
    Scoredepotblue.path
    Startingpath.path
    grabdepotp1.path
    grabdepotbluepart2.path
    intake.path
    intakepart2.path
    red 8.path
    red test1.path
```

### Auto Files (.auto)

Auto files define a sequence of commands to execute. They can include path-following commands, named commands, and parallel/sequential groups.

Example (`Forward.auto`):
```json
{
    "version": "2025.0",
    "command": {
        "type": "sequential",
        "data": {
            "commands": [
                {
                    "type": "path",
                    "data": { "pathName": "Forward Path" }
                }
            ]
        }
    },
    "resetOdom": false,
    "choreoAuto": false
}
```

### Path Files (.path)

Path files define waypoints with positions, headings, and constraints. These are created in the PathPlanner GUI and stored as JSON.

## Creating New Autonomous Routines

### Step 1: Open PathPlanner

Download PathPlanner from [pathplanner.dev](https://pathplanner.dev/) and open the project directory.

### Step 2: Create Paths

1. Go to the **Paths** tab
2. Click **New Path**
3. Place waypoints on the field by clicking
4. Adjust headings by dragging the heading arrows
5. Set constraints (max velocity, acceleration) per segment if needed
6. Save the path (stored as `.path` in `src/main/deploy/pathplanner/paths/`)

### Step 3: Compose Autos

1. Go to the **Autos** tab
2. Click **New Auto**
3. Add path-following commands by referencing saved paths
4. Add named commands (must be registered in code - see below)
5. Use sequential/parallel groups to compose complex routines
6. Save the auto (stored as `.auto` in `src/main/deploy/pathplanner/autos/`)

### Step 4: Register Named Commands (Optional)

If your auto uses named commands (e.g., "shoot", "intake"), register them in code:

```java
NamedCommands.registerCommand("shoot", new InstantCommand(() -> shooter.requestShoot()));
NamedCommands.registerCommand("intake", new InstantCommand(() -> intake.requestIntake()));
```

This should be done before `AutoBuilder.buildAutoChooser()` is called (e.g., in `robotInit()`).

### Step 5: Test

1. Deploy to the robot or run in simulation
2. Select your new auto from the "Auto Chooser" in SmartDashboard
3. Enable autonomous mode
4. Watch the robot follow the path and verify behavior

## Tuning Autonomous Performance

### Path Following PID

If the robot deviates from the planned path:

| Problem | Solution |
|---------|----------|
| Robot consistently lags behind path | Increase translation kP (currently 0.5) |
| Robot overshoots waypoints | Decrease translation kP or add kD |
| Robot heading doesn't match plan | Increase rotation kP (currently 0.3) |
| Robot heading oscillates | Decrease rotation kP or add rotation kD |

### Velocity/Acceleration Limits

Adjust in `SwerveConstants.AutoConstants`:
- `kMaxSpeedMetersPerSecond` = 7.0 m/s (theoretical max for paths)
- `kMaxAccelerationMetersPerSecondSquared` = 2.5 m/s^2

And in PathPlanner GUI's `settings.json`:
- `defaultMaxVel` = 3.084 m/s (conservative default for path generation)
- `defaultMaxAccel` = 5.3 m/s^2

### Starting Position

Set `"resetOdom": true` in the `.auto` file to reset the robot's position to the path's starting waypoint when the auto begins. This is important if the robot's starting position varies between matches.

## PathPlanner Documentation

- [PathPlanner Home](https://pathplanner.dev/)
- [Getting Started Guide](https://pathplanner.dev/pplib-getting-started.html)
- [Java API Reference](https://pathplanner.dev/api/java/)
- [AutoBuilder API](https://pathplanner.dev/api/java/com/pathplanner/lib/auto/AutoBuilder.html)
- [PathPlanner GUI Guide](https://pathplanner.dev/pathplanner-gui.html)
- [Pathfinding](https://pathplanner.dev/pplib-pathfinding.html)
- [GitHub Repository](https://github.com/mjansen4857/pathplanner)
