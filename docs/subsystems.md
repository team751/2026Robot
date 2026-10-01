# Subsystems Reference

This document provides a detailed reference for every subsystem in the robot code.

## Subsystem Index

| Subsystem | Package | Status | Singleton |
|-----------|---------|--------|-----------|
| [SwerveSubsystem](#swervesubsystem) | `frc.robot.subsystems.drive` | Active | Yes |
| [Odometry](#odometry) | `frc.robot.subsystems.drive` | Active | Yes |
| [PhotonVisionSubsystem](#photonvisionsubsystem) | `frc.robot.subsystems.vision` | Active | Yes |
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
| `getPose()` | `Pose2d` | Current fused robot pose. In sim: the CTRE estimator when `SIM_USE_ESTIMATED_POSE` (default), otherwise MapleSim ground truth |
| `getGroundTruthPose()` | `Pose2d` | Sim: MapleSim's true pose (drives `PhotonVisionSim`). Real robot: same as `getPose()` |
| `getChassisSpeeds()` | `ChassisSpeeds` | Current chassis speeds from state |
| `getRobotRelativeSpeeds()` | `ChassisSpeeds` | Speeds from kinematics calculation |
| `resetPose(Pose2d)` | void | Real robot: resets the pose estimate (unchanged from before PhotonVision). Sim: teleports MapleSim and the vision sim (`PhotonVisionSim.onTeleport`), waits 50 ms, resets the estimate and stamps `lastResetFpgaTime` |
| `getLastResetFpgaTime()` | `double` | Sim only: FPGA time of the last pose reset. Odometry rejects vision frames captured before it (`before reset`). Always −∞ on the real robot |
| `setRobotRotationByAlliance()` | void | Circle button: `resetPose((0, 0, alliance heading))` + operator perspective |
| `setOperatorPerspectiveAndAdjustPose(Rotation2d)` | void | Set operator forward direction and rotate the pose by the same delta. Real robot: unchanged from before PhotonVision. Sim: rotates ground truth in place first, then the estimate, separately (never teleports truth onto the estimate) |
| `sysIdQuasistatic(Direction)` | `Command` | SysId quasistatic test |
| `sysIdDynamic(Direction)` | `Command` | SysId dynamic test |

### Periodic Behavior

Every 20ms:
1. Publishes pose (X, Y, rotation) to SmartDashboard
2. Sim only: publishes `Sim/PoseErrorMeters` (fused pose vs MapleSim ground truth)

---

## Odometry

**File**: `frc/robot/subsystems/drive/Odometry.java`
**Extends**: `SubsystemBase`

See [Vision & Odometry Documentation](vision-and-odometry.md) for full details.

### Summary

Filters PhotonVision pose estimates and fuses the good ones into the swerve drive's CTRE pose estimator. Only translation is fused. Heading always comes from the Pigeon 2. Created by `Odometry.getInstance()` in `Robot.robotInit()`, so it fuses from the first loop (disabled and auto included). The constructor is private: `getInstance()` creates `PhotonVisionSubsystem` first, so PV's `periodic()` drains frames before Odometry's runs.

### Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `getInstance()` | `Odometry` | Singleton access (creates `PhotonVisionSubsystem` first) |
| `getPose()` | `Pose2d` | Current fused robot pose |
| `getYaw()` | `Rotation2d` | Heading from the drivetrain (Pigeon) |
| `resetPose(Pose2d)` | void | Delegates to `SwerveSubsystem.resetPose()` |
| `isPoseStable()` | `boolean` | Both cameras have 10+ consecutive accepted estimates within 0.10 m of the pose odometry had at capture time, the latest within the last 0.5 s (dashboard only) |
| `isFrontStable()` / `isSideStable()` | `boolean` | Same, per camera |

### Periodic Behavior

Every 20ms:
1. If the Pigeon yaw rate is above `MAX_ANGULAR_VELOCITY_DPS` (720°/s), skips fusing this loop (`Odometry/VisionRejected` = true)
2. Otherwise, for each cached `VisionObservation` from the front and side cameras:
   - `getRejectReason()`, in order: `non-finite`, `latency`, `before reset`, `not on floor`, `off field`, then single-tag only `ambiguous` and `single tag too far`. On rejection only, the reason goes to `Vision/<Cam>/RejectReason` (so it sticks) and `RejectedCount` goes up.
   - Stability: compares the vision x/y with `drive.samplePoseAt(capture time)` **before** fusing. Within 0.10 m adds 1, otherwise resets to 0.
   - `computeStdDevs()`: multi-tag base if the strategy is multi-tag, unless it's ≤ 2 tags past `FAR_TAG_PAIR_DIST_M` (5 m); otherwise single-tag base. Then × `1 + d²/30`.
   - `drive.addVisionMeasurement(Pose2d(x, y, gyro heading), Utils.fpgaToCurrentTime(ts), stdDevs)`, then `AcceptedCount` goes up
3. Per camera: resets the stability count if nothing was accepted for `POSE_STABLE_TIMEOUT_S` (0.5 s), and publishes `Vision/<Cam>/AcceptedCount` / `RejectedCount`
4. Reads the fused pose from `SwerveSubsystem.getPose()` and updates the `Field2d`
5. Publishes `Odometry/PoseStable`, `Odometry/FrontStable`, `Odometry/SideStable`

### Dependencies

- `SwerveSubsystem` - pose estimator, vision measurement injection, Pigeon yaw rate
- `PhotonVisionSubsystem` - cached `VisionObservation` lists

---

## PhotonVisionSubsystem

**File**: `frc/robot/subsystems/vision/PhotonVisionSubsystem.java`
**Constants**: `PhotonVisionConstants.java`
**Extends**: `SubsystemBase`

See [Vision & Odometry Documentation](vision-and-odometry.md) for full details, the coprocessor bring-up checklist, and on-robot validation.

### Summary

Reads the two PhotonVision cameras (`front` and `side`) on the coprocessor at `10.7.51.11`. In `periodic()`, it drains each camera's `getAllUnreadResults()` exactly once per loop, inside a `try/catch`, so a PV version mismatch (different message format) can't crash the scheduler. Each frame becomes a pose estimate: coprocessor multi-tag first, lowest-ambiguity single tag as the fallback. The estimates are cached as `VisionObservation`s for Odometry, and raw telemetry is published under `Vision/Front` and `Vision/Side` when frames arrive. A camera that is disconnected, or whose drain threw, has its telemetry cleared (empty structs, numbers → NaN) every loop.

### Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `getInstance()` | `PhotonVisionSubsystem` | Singleton access |
| `getFrontObservations()` | `List<VisionObservation>` | Front camera estimates from this loop's frames (no side effects) |
| `getSideObservations()` | `List<VisionObservation>` | Side camera estimates from this loop's frames (no side effects) |

`VisionObservation` is a record: `(EstimatedRobotPose estimate, List<Integer> tagIds, double avgTagDistanceMeters, double ambiguity, double reprojErrPx)`, with `tagCount()` = `tagIds.size()`. These are worked out from the frame, because photonlib's `targetsUsed` lists every target in view:
- `tagIds`: multi-tag uses the result's `fiducialIDsUsed`. Single-tag uses the target photonlib's fallback picked (same rule: lowest ambiguity below 10, skipping -1).
- `avgTagDistanceMeters`: averaged over just those tags. `+Infinity` if none of them are in the frame (Odometry rejects it as `non-finite`).
- `ambiguity`: single-tag pose ambiguity. Always 0 for multi-tag.
- `reprojErrPx`: multi-tag reprojection error in pixels. NaN for single-tag.

`Vision/<Cam>/TagsUsed` draws only `tagIds`, not every tag in view.

### Constants (PhotonVisionConstants.java)

```java
FRONT_CAMERA_NAME = "front"            // must match the PV UI nickname exactly
SIDE_CAMERA_NAME  = "side"
COPROCESSOR_IP    = "10.7.51.11"
FIELD_LAYOUT      = k2026RebuiltWelded // must match the layout selected in the PV UI

// WPILib convention: +X fwd, +Y LEFT, +Z up, +pitch = nose DOWN, +yaw = LEFT
// TODO(measure): placeholders copied from the old Limelight mounts; Y may be mirrored on BOTH
FRONT_CAMERA_OFFSET = (0.0416, -0.1453, 0.4128) m, pitch -30 deg (tilted up)
SIDE_CAMERA_OFFSET  = (-0.2539, 0.1119, 0.2906) m, yaw -90 deg (sign unverified)

SINGLE_TAG_STD_DEVS = (4.0, 4.0, 99999)    // scaled by 1 + d^2 / STD_DEV_DISTANCE_DIVISOR (30)
MULTI_TAG_STD_DEVS  = (0.5, 0.5, 99999)
FAR_TAG_PAIR_DIST_M = 5.0                  // multi-tag with <= 2 tags past this gets single-tag trust
MAX_AMBIGUITY = 0.2, MAX_SINGLE_TAG_DIST_M = 4.0           // single-tag only
MAX_Z_ERROR_M = 0.25, MAX_LATENCY_S = 0.5, FIELD_BORDER_MARGIN_M = 0.5
MAX_ANGULAR_VELOCITY_DPS = 720
POSE_STABLE_EPSILON_METERS = 0.10, POSE_STABLE_THRESHOLD = 10, POSE_STABLE_TIMEOUT_S = 0.5
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
