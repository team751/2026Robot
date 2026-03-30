# Architecture Overview

This document describes the high-level architecture of the robot code, the control flow, and the design patterns used throughout the codebase.

## System Diagram

```
Main.java
  └─ Robot.java (TimedRobot, 50Hz)
       ├─ CommandScheduler (runs all subsystem periodic + commands)
       │    ├─ SwerveSubsystem (drive control, PathPlanner AutoBuilder)
       │    │    └─ [sim] MapleSimSwerveDrivetrain (200Hz physics)
       │    ├─ Odometry (vision + wheel fusion, Field2d)
       │    │    └─ LimelightSubsystem (dual camera AprilTag poses)
       │    ├─ Superstructure (state machine coordinator)
       │    └─ [future subsystems...]
       ├─ ControlBoard (PS5 controller bindings → SwerveRequests)
       └─ TunableParameter.updateAll() (SmartDashboard polling)
```

## Robot Lifecycle

The robot program follows WPILib's `TimedRobot` lifecycle. The main loop runs at **50Hz** (20ms period).

### Initialization Sequence

```
1. JVM starts → Main.main() → RobotBase.startRobot(Robot::new)
2. Robot() constructor:
   a. Odometry.getInstance()         ← creates Odometry → SwerveSubsystem → LimelightSubsystem
   b. CommandScheduler.getInstance() ← gets the singleton scheduler
   c. SwerveSubsystem.getInstance() ← already created by Odometry
   d. ControlBoard.getInstance()     ← creates controllers (wrapped in try/catch)
3. robotInit():
   a. Port forwarding for Limelight cameras (5800-5809)
   b. AutoBuilder.buildAutoChooser() → SmartDashboard "Auto Chooser"
```

**Important**: The initialization order matters. `Odometry` creates `SwerveSubsystem` and `LimelightSubsystem` as dependencies. The `ControlBoard` depends on `SwerveSubsystem` being initialized.

### Periodic Loop (50Hz)

Every 20ms, WPILib calls:

```
robotPeriodic():
  1. TunableParameter.updateAll()    ← polls SmartDashboard for changed values
  2. CommandScheduler.run()          ← runs all registered subsystem periodic() + scheduled commands
     ├─ SwerveSubsystem.periodic()  ← operator perspective, publishes pose
     ├─ Odometry.periodic()         ← vision fusion, Field2d update
     ├─ Superstructure.periodic()   ← state machine transitions
     └─ [active commands execute]
  3. ControlBoard.displayUI()        ← (currently empty, reserved for dashboard updates)
```

### Mode Transitions

| Event | Method Called | What Happens |
|-------|-------------|--------------|
| DS connects | `driverStationConnected()` | `ControlBoard.tryInit()` — binds controllers if not already done |
| Auto starts | `autonomousInit()` | Schedules selected auto from `autoChooser` |
| Auto ends | `autonomousExit()` | Cancels running auto command |
| Teleop starts | `teleopInit()` | Initializes `LimelightSubsystem`, sets alliance perspective |
| Disabled | `disabledInit()` | Stops `SignalLogger` |

## Design Patterns

### 1. Singleton Subsystems

Every subsystem uses the singleton pattern to ensure exactly one instance exists:

```java
public class MySubsystem extends SubsystemBase {
    private static MySubsystem instance;

    public static MySubsystem getInstance() {
        if (instance == null) instance = new MySubsystem();
        return instance;
    }

    private MySubsystem() {
        // Private constructor prevents external instantiation
    }
}
```

**Why**: Subsystems represent physical hardware (one drivetrain, one shooter). Multiple instances would conflict on hardware resources. The singleton pattern also enables any class to access a subsystem without dependency injection.

### 2. Request-Based State Machines

Subsystems that have multiple operating modes use a request-based state machine pattern:

```java
// State definition
enum SubsystemState { IDLE, ACTIVE, ... }

// Request fields
boolean requestIdle = false;
boolean requestActive = false;

// Request methods (public API)
public void requestIdle() {
    unsetAllRequests();
    requestIdle = true;
}

// State transitions happen in periodic()
@Override
public void periodic() {
    SubsystemState nextState = currentState;
    if (requestIdle) nextState = SubsystemState.IDLE;
    else if (requestActive) nextState = SubsystemState.ACTIVE;

    if (nextState != currentState) {
        currentState = nextState;
        unsetAllRequests();
        // Apply state-specific behavior
    }
}
```

**Why**: This decouples "what we want to do" from "when the transition happens." Commands and the Superstructure call request methods; the subsystem decides when and how to transition. This prevents race conditions and ensures transitions are atomic within a single periodic cycle.

**Used by**: `Superstructure` (PRE_HOME/IDLE), `ShooterSubsystem` (IDLE/SPINNING, commented out).

### 3. Superstructure Coordinator

The `Superstructure` is a meta-subsystem that coordinates multiple subsystems through a single state machine. It holds references to all mechanism subsystems and orchestrates their states together.

```java
public class Superstructure extends SubsystemBase {
    private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
    // private final ShooterSubsystem shooterSubsystem = ShooterSubsystem.getInstance();

    enum SuperstructureState { PRE_HOME, IDLE }

    @Override
    public void periodic() {
        // Transitions between states, commanding subsystems as needed
    }
}
```

**Why**: In complex robots, multiple subsystems must move in coordination (e.g., elevator up THEN wrist rotate THEN shoot). The Superstructure enforces safe sequencing and prevents conflicting states.

### 4. Command-Based Framework

This project uses WPILib's [command-based framework](https://docs.wpilib.org/en/stable/docs/software/commandbased/index.html):

- **Subsystems** own hardware and define capabilities
- **Commands** compose subsystem actions into behaviors
- **CommandScheduler** manages command lifecycle (scheduling, interruption, requirements)
- **Triggers** (button presses, conditions) schedule commands

Currently, most logic lives directly in subsystem `periodic()` methods and the `ControlBoard` bindings. As the robot grows in complexity, more logic will move into dedicated `Command` classes.

### 5. CAN Bus Architecture

Two CAN buses separate high-bandwidth swerve traffic from other devices:

```
CAN Bus "rio" (Robot.riobus)         CAN Bus "Drivebus" (Robot.drivebus)
├─ General devices                    ├─ FL Drive Motor (10)
├─ Shooter Motor (15, planned)        ├─ FL Steer Motor (11)
└─ Future subsystem motors            ├─ FL CANcoder (12)
                                      ├─ FR Drive Motor (20)
                                      ├─ FR Steer Motor (21)
                                      ├─ FR CANcoder (22)
                                      ├─ BR Drive Motor (30)
                                      ├─ BR Steer Motor (31)
                                      ├─ BR CANcoder (32)
                                      ├─ BL Drive Motor (40)
                                      ├─ BL Steer Motor (41)
                                      ├─ BL CANcoder (42)
                                      └─ Pigeon2 IMU (2)
```

**Why**: Swerve drives generate heavy CAN traffic (8 motors + 4 encoders + 1 gyro updating at 250Hz). A dedicated CAN bus prevents this from starving other devices of bandwidth.

Always use `Robot.riobus` and `Robot.drivebus` static fields. The string constants in `frc.robot.util.Constants` are deprecated.

## File Organization

### Subsystem Package Convention

Each subsystem is organized in its own package:

```
subsystems/
  drive/
    SwerveSubsystem.java     # Main subsystem class
    SwerveConstants.java      # Constants for this subsystem
    Odometry.java             # Related helper subsystem
    generated/
      TunerConstants.java     # Auto-generated, do not edit
  vision/
    LimelightSubsystem.java
    LimelightConstants.java
  shooter/
    ShooterSubsystem.java
    ShooterConstants.java
```

### Library vs Robot Code

- `frc.lib` — Reusable utilities not specific to this robot (could be used on any FRC robot)
- `frc.robot` — Robot-specific code
- `frc.robot.util` — Robot-specific utilities (ControlBoard, FieldConstants)
- `org.ironmaple.simulation` — Third-party simulation code bundled in-tree

## Data Flow

### Telemetry Pipeline

```
Subsystems → SmartDashboard/NetworkTables → Dashboard (Elastic/Shuffleboard/AdvantageScope)
```

Key telemetry published:
- `Swerve/Pose x`, `Swerve/Pose y`, `Swerve/Rotation` — robot pose from SwerveSubsystem
- `Odometry/X`, `Odometry/Y`, `Odometry/Rotation` — robot pose from Odometry
- `Pigeon Yaw/Pitch/Roll` — IMU readings
- `Interpolating?` — whether both Limelights have AprilTag targets
- `Superstructure/loopCycleTime` — state machine timing
- `Field` — Field2d visualization data
- `DriveState/*` — full swerve module states (simulation telemetry)
- `FieldSimulation/Fuel` — game piece positions (simulation only)
