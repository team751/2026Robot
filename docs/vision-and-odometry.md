# Vision & Odometry

This document covers the Limelight vision system, AprilTag-based localization, odometry fusion, and how the robot knows where it is on the field.

## How the Robot Knows Where It Is

The robot determines its position on the field using two complementary systems:

### 1. Wheel Odometry (Dead Reckoning)

The swerve drive measures every rotation of its wheels and uses kinematics to estimate how far the robot has moved. This is handled internally by CTRE's `SwerveDrivetrain` class.

**Pros**: Always available, very high update rate (250Hz on CAN FD)
**Cons**: Drifts over time due to wheel slip, collisions, and encoder error

### 2. Vision (AprilTag Localization)

Two Limelight cameras detect [AprilTags](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html) placed around the field. Each AprilTag has a known position on the field, so when the camera sees one, it can calculate the robot's position.

**Pros**: Provides absolute position (no drift), corrects accumulated wheel odometry error
**Cons**: Only available when AprilTags are visible, lower update rate, susceptible to lighting issues

### 3. Sensor Fusion

`Odometry.java` combines both systems: wheel odometry provides continuous tracking, and vision measurements periodically correct the estimate. This is done via `SwerveSubsystem.addVisionMeasurement()`, which uses WPILib's pose estimator under the hood.

**Important**: Only the **translation** (x, y) from vision is used. The **heading** (rotation) always comes from the Pigeon 2 gyro, which is more reliable than vision-derived heading.

## Limelight Configuration

### Camera Specifications

| Property | Front Camera | Back Camera |
|----------|-------------|-------------|
| Name | `limelight-front` | `limelight-back` |
| Model | Limelight 3G | Limelight 2 |
| IP Address | 10.7.51.71 | 10.7.51.75 |
| Stream URL | http://10.7.51.71:5800 | http://10.7.51.75:5800 |
| Dashboard URL | http://10.7.51.71:5801 | http://10.7.51.75:5801 |

### Camera Positions (Robot Frame)

Positions are measured from the center of the robot:

| Axis | Front Camera | Back Camera | Convention |
|------|-------------|-------------|------------|
| X (forward) | 0.225 m | -0.01 m | Positive = forward |
| Y (left) | 0.025 m | -0.29 m | Positive = left |
| Z (up) | 0.235 m | 0.285 m | Positive = up |
| Rotation | None (0,0,0) | Yaw = 90 degrees | |

These offsets are defined in `LimelightConstants.java` and are critical for accurate pose estimation. If a camera is moved, these must be re-measured and updated.

### Port Forwarding

In `Robot.robotInit()`, ports 5800-5809 are forwarded from `limelight.local` so the Limelight web interface is accessible when connected to the robot via USB or ethernet:

```java
for (int port = 5800; port <= 5809; port++) {
    PortForwarder.add(port, "limelight.local", port);
}
```

## LimelightSubsystem.java

**File**: `frc/robot/subsystems/vision/LimelightSubsystem.java`

The singleton `LimelightSubsystem` initializes both Limelight cameras and provides methods to get AprilTag-derived robot poses.

### Initialization

The constructor:
1. Creates `Limelight` objects (from the YALL library) using camera names
2. Configures camera offsets (position relative to robot center) on each camera

### Key Methods

| Method | Returns | Description |
|--------|---------|-------------|
| `getInstance()` | `LimelightSubsystem` | Singleton access |
| `getBotPoseFront()` | `Pose2d` or `null` | Robot pose from front camera (WPILib Blue origin) |
| `getBotPoseBack()` | `Pose2d` or `null` | Robot pose from back camera (WPILib Blue origin) |
| `getBotPoseInterpolated()` | `Pose2d` | Averaged pose from both cameras (50% interpolation) |
| `hasTarget()` | `boolean` | True if BOTH cameras see AprilTags |
| `getAprilTagId()` | `int` | ID of the tag the front camera is tracking |
| `robotInit()` | void | Sets up camera streams for SmartDashboard |

### Pose Estimation Flow

```
1. LimelightHelpers.SetRobotOrientation() ← feed gyro heading to Limelight
2. LimelightHelpers.getBotPoseEstimate_wpiBlue() ← get MegaTag2 pose estimate
3. If pose is (0,0), return null (no tag visible)
4. Return the estimated Pose2d
```

**Note**: `SetRobotOrientation` feeds the gyro heading to the Limelight before getting the pose estimate. This enables MegaTag2, which uses the known heading to improve the accuracy of the position estimate.

### Interpolation

When both cameras have targets, `getBotPoseInterpolated()` averages them using WPILib's `Pose2d.interpolate()` at t=0.5. When only one camera has a target, that camera's estimate is used directly.

## Odometry.java

**File**: `frc/robot/subsystems/drive/Odometry.java`

Fuses vision measurements into the swerve drive's pose estimator and publishes telemetry.

### What It Does Every Periodic Cycle

```java
@Override
public void periodic() {
    // 1. Publish current pose to SmartDashboard
    SmartDashboard.putNumber("Odometry/X", robotPose.getX());
    SmartDashboard.putNumber("Odometry/Y", robotPose.getY());
    SmartDashboard.putNumber("Odometry/Rotation", robotPose.getRotation().getDegrees());

    // 2. Feed vision measurements to swerve drive
    if (limelights.getBotPoseFront() != null) {
        drive.addVisionMeasurement(
            new Pose2d(front.getTranslation(), drive.getPose().getRotation()), // Use gyro heading
            currentTime
        );
    }
    if (limelights.getBotPoseBack() != null) {
        drive.addVisionMeasurement(
            new Pose2d(back.getTranslation(), drive.getPose().getRotation()), // Use gyro heading
            currentTime
        );
    }

    // 3. Get fused pose from swerve drive
    robotPose = drive.getPose();

    // 4. Update Field2d visualization
    field.setRobotPose(robotPose);

    // 5. Publish Pigeon IMU data
    SmartDashboard.putNumber("Pigeon Yaw", drive.getPigeon2().getYaw().getValueAsDouble());
    SmartDashboard.putNumber("Pigeon Pitch", drive.getPigeon2().getPitch().getValueAsDouble());
    SmartDashboard.putNumber("Pigeon Roll", drive.getPigeon2().getRoll().getValueAsDouble());
}
```

### Key Design Decision: Vision Translation Only

Notice the vision measurement construction:
```java
new Pose2d(visionPose.getTranslation(), drive.getPose().getRotation())
```

This uses the **translation (x, y) from vision** but keeps the **rotation from the gyro**. This is because:
- The Pigeon 2 gyro provides very accurate heading
- Vision-derived heading can be noisy, especially with single-tag views
- Using gyro heading prevents the robot from "spinning" its estimated heading when AprilTags are partially visible

### SmartDashboard Telemetry

| Key | Value |
|-----|-------|
| `Odometry/X` | Robot X position (meters) |
| `Odometry/Y` | Robot Y position (meters) |
| `Odometry/Rotation` | Robot heading (degrees) |
| `Pigeon Yaw` | Gyro yaw (degrees) |
| `Pigeon Pitch` | Gyro pitch (degrees) |
| `Pigeon Roll` | Gyro roll (degrees) |
| `Swerve Rotation` | Heading from Rotation3d.getZ() (degrees) |
| `Interpolating?` | Whether both cameras have targets |
| `Field` | Field2d visualization data |

## LimelightHelpers.java

**File**: `frc/robot/util/LimelightHelpers.java`

This is the official Limelight helper library (v1.14, requires LLOS 2026.0 or later). It provides static methods for interacting with Limelight cameras via NetworkTables.

### Key Static Methods

| Method | Description |
|--------|-------------|
| `getTV(name)` | Returns true if the camera has a valid target |
| `getFiducialID(name)` | Returns the ID of the primary tracked fiducial (AprilTag) |
| `getBotPoseEstimate_wpiBlue(name)` | Gets robot pose in WPILib Blue-origin field coordinates |
| `SetRobotOrientation(name, yaw, ...)` | Feeds robot orientation to Limelight for MegaTag2 |

### YALL (Yet Another Limelight Library)

In addition to `LimelightHelpers`, the project uses the **YALL** vendordep (v2026.1.12). This provides the `Limelight` class used in `LimelightSubsystem` for camera initialization and settings configuration. Both libraries coexist - YALL for camera setup, LimelightHelpers for pose data.

## Tuning Vision

### When Vision Estimates Are Poor

1. **Check camera exposure** - Access the Limelight web interface and adjust exposure/gain
2. **Verify camera offsets** - Measure physical camera position and update `LimelightConstants.java`
3. **Adjust standard deviations** - Currently commented out in `Odometry.java`:
   ```java
   // drive.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
   ```
   Lower values = trust vision more. The `9999999` for rotation means "don't use vision rotation."
4. **Check AprilTag placement** - Ensure field AprilTags match the expected layout

### Limelight Documentation

- [Limelight Docs](https://docs.limelightvision.io/)
- [Getting Started](https://docs.limelightvision.io/docs/docs-limelight/getting-started/summary)
- [FRC Networking](https://docs.limelightvision.io/docs/docs-limelight/getting-started/FRC/networking)
- [Limelight Lib API](https://docs.limelightvision.io/docs/docs-limelight/apis/limelight-lib)
