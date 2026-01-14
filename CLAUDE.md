# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a WPILib-based FRC (FIRST Robotics Competition) 2026 robot codebase written in Java. The robot uses CTRE Phoenix 6's generated swerve drivetrain and Limelight for AprilTag-based localization.

## Build and Development Commands

### Building and Deploying
```bash
# Build the project
./gradlew build

# Deploy to the robot (RoboRIO)
./gradlew deploy

# Build and deploy
./gradlew build deploy
```

### Code Quality
```bash
# Check code formatting
./gradlew spotlessCheck

# Auto-format code
./gradlew spotlessApply
```

### Testing
```bash
# Run all tests
./gradlew test

# Run tests for a specific class
./gradlew test --tests ClassName
```

### Simulation
```bash
# Run robot code in simulation with GUI
./gradlew simulateJava
```

## Code Architecture

### Singleton Pattern
All major subsystems and utilities use the singleton pattern with `getInstance()`. This ensures only one instance exists during runtime. When adding new subsystems, follow this pattern:
```java
private static MySubsystem instance = null;
public static MySubsystem getInstance() {
    if (instance == null) instance = new MySubsystem();
    return instance;
}
```

### Subsystem Organization
The robot follows WPILib's command-based architecture:

- **Robot.java**: Main entry point, manages autonomous and teleop modes, initializes subsystems
- **Superstructure.java**: State machine that coordinates between subsystems (IntakeSubsystem, ClimbSubsystem, SwerveSubsystem, LightsSubsystem)
- **Subsystems**: Each subsystem manages a specific robot mechanism

Key subsystems:
- `SwerveSubsystem`: CTRE Phoenix 6 generated swerve drive with PathPlanner integration
- `LimelightSubsystem`: Vision processing using Limelight for AprilTag detection
- `AutonSubsystem`: Autonomous routine management using Choreo
- `IntakeSubsystem`, `ClimbSubsystem`, `LightsSubsystem`: Mechanism control

### Swerve Drive Implementation
The swerve drivetrain extends CTRE's `TunerSwerveDrivetrain` class (generated code in `subsystems/drive/generated/TunerConstants.java`). The robot uses:
- Two CAN buses: "rio" and "Drivebus" (configured in Constants.java and Robot.java)
- Field-centric control as the default drive mode
- PathPlanner's `AutoBuilder` for autonomous path following
- LocalADStar pathfinding algorithm

### Control System
`ControlBoard.java` manages all operator input:
- Singleton pattern for driver and operator PS5 controllers
- Lazy initialization via `tryInit()` method called when driver station connects
- Driver controls: Field-centric swerve drive with precise control mode (right bumper)
- Default command pattern: Swerve drive's default command reads driver input continuously

### Vision and Localization
`LimelightSubsystem` handles vision:
- Wraps Limelight API for AprilTag detection
- Provides estimated robot pose from AprilTags
- Alliance-specific tag filtering configured on driver station connection
- Port forwarding (5800-5809) configured in Robot.robotInit() for network access

### Autonomous
`AutonSubsystem` uses Choreo for trajectory generation:
- `AutoFactory` creates routines that reset odometry and follow paths
- Trajectories are loaded from `src/main/deploy/choreo/`
- Auto chooser displayed on SmartDashboard for driver selection
- PathPlanner integration for dynamic pathfinding during auto

### Tunable Parameters
`TunableParameter` class provides runtime tuning via SmartDashboard:
- Register parameters with name, initial value, and callback
- `TunableParameter.updateAll()` called in `Robot.robotPeriodic()` to sync values
- Changes on SmartDashboard trigger callbacks automatically

## Important Patterns

### CAN Bus Configuration
The robot uses two CAN buses:
```java
public static final CANBus riobus = new CANBus("rio");
public static final CANBus drivebus = new CANBus(Constants.drivebus);
```
When adding new CTRE devices, specify the correct bus.

### Command Scheduler Debugging
Command lifecycle logging is enabled in Robot.java for development. Disable for competitions:
```java
scheduler.onCommandInitialize(...)
scheduler.onCommandFinish(...)
```

### Signal Logging
CTRE Signal Logger is configured but commented out:
```java
SignalLogger.setPath("/media/sda1/");
// SignalLogger.start();  // Enable for competition
```

### Operator Perspective
The swerve subsystem automatically sets the operator perspective based on alliance color (red = 180°, blue = 0°).

## Vendor Dependencies

Located in `vendordeps/`:
- Phoenix6-26.1.0.json (CTRE Phoenix 6)
- PathplannerLib-2026.1.2.json
- ChoreoLib2026.json
- REVLib.json
- yall.json (Limelight library)

Note: MapleSimLib is mentioned in README as missing but simulation code is commented out.

## Code Style

The project uses Spotless with:
- Google Java Format
- Tab indentation (2 spaces)
- Automatic import organization
- Trailing whitespace removal

Always run `./gradlew spotlessApply` before committing.

## Testing

The project is configured for JUnit 5 (Jupiter). Test files should:
- Use `@Test` annotations
- Be placed in corresponding test directories
- Run with `./gradlew test`
