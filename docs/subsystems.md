# Subsystems Reference

This document provides a detailed reference for every subsystem in the robot code.

## Subsystem Index

| Subsystem | Package | Status | Singleton |
|-----------|---------|--------|-----------|
| [SwerveSubsystem](#swervesubsystem) | `frc.robot.subsystems.drive` | Active | Yes |
| [Odometry](#odometry) | `frc.robot.subsystems.drive` | Active | Yes |
| [LimelightSubsystem](#limelightsubsystem) | `frc.robot.subsystems.vision` | Active | Yes |
| [Superstructure](#superstructure) | `frc.robot.subsystems` | Active (skeleton) | Yes |
| [ShooterSubsystem](#shootersubsystem) | `frc.robot.subsystems.shooter` | Commented out | Yes |

## SwerveSubsystem

**File**: `frc/robot/subsystems/drive/SwerveSubsystem.java`
**Extends**: `TunerSwerveDrivetrain` (CTRE Phoenix 6) + implements `Subsystem` (WPILib)
**Constants**: `SwerveConstants.java`, `TunerConstants.java` (generated)

See [Swerve Drive Documentation](swerve-drive.md) for full details.

### Summary

The swerve drive subsystem controls the robot's 4-module swerve drivetrain. It:
- Manages all 8 drive/steer motors and 4 CANcoders
- Provides pose estimation (real or simulated)
- Configures PathPlanner's AutoBuilder for autonomous
- Manages alliance-aware operator perspective
- Supports SysId characterization
- Starts MapleSim simulation thread when in sim mode

### Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `getInstance()` | `SwerveSubsystem` | Singleton access |
| `applyRequest(Supplier<SwerveRequest>)` | `Command` | Command that continuously applies a swerve request |
| `getPose()` | `Pose2d` | Current robot pose (sim-aware) |
| `getChassisSpeeds()` | `ChassisSpeeds` | Current chassis speeds from state |
| `getRobotRelativeSpeeds()` | `ChassisSpeeds` | Speeds from kinematics calculation |
| `resetPose(Pose2d)` | void | Reset odometry to specified pose (sim-aware) |
| `bigResetPose()` | void | Reset odometry to origin (0,0,0) |
| `setOperatorPerspectiveAndAdjustPose(Rotation2d)` | void | Set operator forward direction with pose correction |
| `sysIdQuasistatic(Direction)` | `Command` | SysId quasistatic test |
| `sysIdDynamic(Direction)` | `Command` | SysId dynamic test |

### Periodic Behavior

Every 20ms:
1. Checks if operator perspective needs updating (while disabled or on first enable)
2. Publishes pose (X, Y, rotation) to SmartDashboard

---

## Odometry

**File**: `frc/robot/subsystems/drive/Odometry.java`
**Extends**: `SubsystemBase`

See [Vision & Odometry Documentation](vision-and-odometry.md) for full details.

### Summary

Fuses Limelight vision measurements with swerve wheel odometry to provide accurate robot positioning. Publishes telemetry to SmartDashboard.

### Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `getInstance()` | `Odometry` | Singleton access |
| `getPose()` | `Pose2d` | Current fused robot pose |
| `resetPose(Pose2d)` | void | Delegates to `SwerveSubsystem.resetPose()` |

### Periodic Behavior

Every 20ms:
1. Publishes `Odometry/X`, `Odometry/Y`, `Odometry/Rotation` to SmartDashboard
2. Feeds front camera vision measurement to swerve (if available)
3. Feeds back camera vision measurement to swerve (if available)
4. Gets fused pose from swerve drive
5. Updates `Field2d` visualization
6. Publishes Pigeon IMU yaw/pitch/roll
7. Publishes `Interpolating?` boolean

### Dependencies

- `SwerveSubsystem` - for pose estimation and vision measurement injection
- `LimelightSubsystem` - for AprilTag-derived robot poses

---

## LimelightSubsystem

**File**: `frc/robot/subsystems/vision/LimelightSubsystem.java`
**Constants**: `LimelightConstants.java`
**Extends**: `SubsystemBase`

See [Vision & Odometry Documentation](vision-and-odometry.md) for full details.

### Summary

Manages two Limelight cameras for AprilTag-based robot localization. Provides robot pose estimates in WPILib Blue-origin field coordinates.

### Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `getInstance()` | `LimelightSubsystem` | Singleton access |
| `getBotPoseFront()` | `Pose2d` or `null` | Robot pose from front camera |
| `getBotPoseBack()` | `Pose2d` or `null` | Robot pose from back camera |
| `getBotPoseInterpolated()` | `Pose2d` | Average of both cameras (50% interpolation) |
| `hasTarget()` | `boolean` | True if **both** cameras see AprilTags |
| `getAprilTagId()` | `int` | ID of primary tag (front camera) |
| `robotInit()` | void | Starts camera streams for SmartDashboard |

### Constants (LimelightConstants.java)

```java
// Front Camera (Limelight 3G)
LimelightFront.name = "limelight-front"
LimelightFront.streamIp = "http://10.7.51.71:5800"
LimelightFront.dashboardIp = "http://10.7.51.71:5801"
LimelightFront.xOffset = 0.225m   // forward
LimelightFront.yOffset = 0.025m   // left
LimelightFront.zOffset = 0.235m   // up

// Back Camera (Limelight 2)
LimelightBack.name = "limelight-back"
LimelightBack.streamIp = "http://10.7.51.75:5800"
LimelightBack.dashboardIp = "http://10.7.51.75:5801"
LimelightBack.xOffset = -0.01m    // slightly backward
LimelightBack.yOffset = -0.29m    // right
LimelightBack.zOffset = 0.285m    // up
LimelightBack.rotationOffset = 90 degrees yaw
```

---

## Superstructure

**File**: `frc/robot/subsystems/Superstructure.java`
**Extends**: `SubsystemBase`

### Summary

The Superstructure is a **coordinator subsystem** that manages the overall robot state through a central state machine. It holds references to other subsystems and orchestrates their behavior.

Currently a skeleton with two states — this will grow as mechanisms are added.

### States

| State | Description |
|-------|-------------|
| `PRE_HOME` | Initial state before homing procedures complete |
| `IDLE` | Normal idle state, mechanisms at rest |

### Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `getInstance()` | `Superstructure` | Singleton access |
| `requestHome()` | void | Request transition to homing |
| `requestIdle()` | void | Request transition to idle |
| `unsetAllRequests()` | void | Clear all pending state requests |

### State Machine Pattern

```java
@Override
public void periodic() {
    // Track timing
    double time = RobotController.getFPGATime();
    SmartDashboard.putNumber("Superstructure/loopCycleTime", time - lastFPGATimestamp);

    // Determine next state based on requests
    SuperstructureState nextState = systemState;
    switch (systemState) {
        case PRE_HOME -> { /* transition logic */ }
        case IDLE -> { /* transition logic */ }
    }

    // Apply transition
    if (nextState != systemState) {
        mStateStartTime = time;
        systemState = nextState;
    }
}
```

### How to Extend

When adding new mechanisms, the Superstructure should:
1. Hold a reference to each mechanism subsystem
2. Add new states (e.g., `SCORING`, `CLIMBING`, `INTAKING`)
3. In each state's case block, command subsystems to their appropriate states
4. Enforce safe sequencing (e.g., elevator up BEFORE wrist extends)

Example future pattern:
```java
case SCORING -> {
    elevatorSubsystem.requestExtend();
    if (elevator.isAtTarget()) {
        wristSubsystem.requestScore();
        shooterSubsystem.requestShoot();
    }
    if (requestIdle) nextState = IDLE;
}
```

### Dependencies

- `SwerveSubsystem` - reference held, not currently commanded
- (Future) `ShooterSubsystem`, elevator, wrist, etc.

---

## ShooterSubsystem (Commented Out)

**File**: `frc/robot/subsystems/shooter/ShooterSubsystem.java`
**Constants**: `ShooterConstants.java`
**Status**: Entirely commented out

### Summary

A complete but inactive subsystem for controlling a shooter mechanism. The entire file is commented out, likely because the physical shooter isn't built yet or the design changed.

### Design (When Active)

**Hardware:**
- 1x TalonFX motor (CAN ID 15, on `Robot.drivebus`)
- VoltageOut control mode
- 40A stator current limit
- Counter-clockwise positive motor direction

**State Machine:**
| State | Motor Voltage | Description |
|-------|--------------|-------------|
| `IDLE` | 0V | Motor stopped |
| `SPINNING` | 12V | Full shoot speed |

**Constants:**
```java
shooterSpeed = 12.0    // Voltage for shooting
spitSpeed = 4.0        // Voltage for spitting (negated)
```

**PID Gains** (all zeros — using voltage control, not closed-loop):
```java
kP = 0, kI = 0, kD = 0, kS = 0, kA = 0, kV = 0, kG = 0
```

### How to Re-Enable

1. Uncomment `ShooterSubsystem.java` and `ShooterConstants.java`
2. Uncomment the import/reference in `Superstructure.java`
3. Uncomment the import/reference in `ControlBoard.java`
4. Initialize in `Robot.java` constructor: `ShooterSubsystem.getInstance();`
5. Add controller bindings in `ControlBoard.configureOperatorBindings()`
6. Tune CAN ID, current limits, and motor direction for the actual hardware

## Adding a New Subsystem

### Template

```java
package frc.robot.subsystems.mysubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class MySubsystem extends SubsystemBase {
    private static MySubsystem instance;

    // Hardware
    // private final TalonFX motor = MySubsystemConstants.motorConfig.createDevice(TalonFX::new);

    // State machine
    private enum State { IDLE, ACTIVE }
    private State state = State.IDLE;
    private boolean requestIdle = false;
    private boolean requestActive = false;

    public static MySubsystem getInstance() {
        if (instance == null) instance = new MySubsystem();
        return instance;
    }

    private MySubsystem() {
        // Initialize hardware, set default states
    }

    @Override
    public void periodic() {
        State nextState = state;
        if (requestIdle) nextState = State.IDLE;
        else if (requestActive) nextState = State.ACTIVE;

        if (nextState != state) {
            state = nextState;
            unsetAllRequests();
            switch (state) {
                case IDLE -> { /* stop motors */ }
                case ACTIVE -> { /* run motors */ }
            }
        }

        // Telemetry
        SmartDashboard.putString("MySubsystem/State", state.toString());
    }

    private void unsetAllRequests() {
        requestIdle = false;
        requestActive = false;
    }

    public void requestIdle() { unsetAllRequests(); requestIdle = true; }
    public void requestActive() { unsetAllRequests(); requestActive = true; }
}
```

### Integration Checklist

- [ ] Create subsystem class with singleton pattern
- [ ] Create constants file with hardware configuration
- [ ] Add reference in `Superstructure.java`
- [ ] Initialize singleton in `Robot.java` constructor
- [ ] Add controller bindings in `ControlBoard.java`
- [ ] Add SmartDashboard telemetry in `periodic()`
- [ ] Test in simulation before deploying
