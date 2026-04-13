# Simulation

This document covers how the physics simulation works, how to run it, and how to use AdvantageScope for visualization.

## Overview

The robot code supports full physics simulation using **MapleSim**, a simulation framework built on the [dyn4j](https://dyn4j.org/) 2D rigid-body physics engine. When running in simulation mode, MapleSim provides realistic swerve drive physics, field collisions, game piece interactions, and motor behavior.

The simulation runs at **200Hz** (0.005s period) on a dedicated thread, separate from the main robot loop (50Hz).

## Running Simulation

```bash
# Launch simulation
./gradlew simulateJava
```

This starts:
1. The robot code in simulation mode
2. A **Simulation GUI** window for controlling robot state
3. A **simulated Driver Station** connection

### Using the Simulation GUI

1. **Select robot mode**: Drag "Teleoperated" or "Autonomous" to enable those modes
2. **Connect a controller**: Plug in a gamepad to drive the robot in teleop
3. **View in AdvantageScope**: Open AdvantageScope and connect to `localhost` to visualize

### Visualization with AdvantageScope

[AdvantageScope](https://docs.advantagescope.org/) is the recommended visualization tool for simulation:

1. Download from [GitHub Releases](https://github.com/Mechanical-Advantage/AdvantageScope/releases)
2. Open AdvantageScope
3. Connect to the running simulation via **File > Connect to Simulator** (or `localhost`)
4. Use the **3D Field** tab to see the robot and game pieces
5. Use the **2D Field** tab with the `Field` NetworkTable entry to see the robot pose

**Published NetworkTables data:**
- `DriveState/Pose` - Robot pose (Pose2d struct)
- `DriveState/Speeds` - Chassis speeds (ChassisSpeeds struct)
- `DriveState/ModuleStates` - Current swerve module states
- `DriveState/ModuleTargets` - Target swerve module states
- `DriveState/ModulePositions` - Module positions for odometry
- `DriveState/Timestamp` - Odometry timestamp
- `DriveState/OdometryFrequency` - Odometry update frequency
- `Field/robotPose` - Robot pose as double array [x, y, deg]
- `FieldSimulation/Fuel` - Game piece positions (Pose3d array)

## Architecture

### How Simulation Starts

```
SwerveSubsystem constructor
  └─ if (Utils.isSimulation()) startSimThread()
       ├─ Creates MapleSimSwerveDrivetrain
       │    ├─ Builds DriveTrainSimulationConfig (mass, bumpers, gyro, modules)
       │    ├─ Creates SwerveDriveSimulation (physics body)
       │    ├─ Creates IntakeSimulation (game piece pickup)
       │    ├─ Registers with SimulatedArena
       │    └─ Adds sample game piece (Fuel at 1,1)
       └─ Starts Notifier at 200Hz → calls simDrivetrain.update()
```

### Simulation Update Loop (200Hz)

Every 5ms, the simulation:

```java
public void update() {
    // 1. Step the physics simulation (all bodies, collisions, forces)
    SimulatedArena.getInstance().simulationPeriodic();

    // 2. Inject simulated gyro values from physics
    pigeonSim.setRawYaw(mapleSimDrive.getSimulatedDriveTrainPose().getRotation());
    pigeonSim.setAngularVelocityZ(chassisSpeeds.omegaRadiansPerSecond);
}
```

Each `SimSwerveModule` also injects motor/encoder sim state:
- Drive motor: position, velocity, supply voltage → reads motor voltage command
- Steer motor: position, velocity, supply voltage → reads motor voltage command
- CANcoder: position, velocity, supply voltage

### Key Simulation Classes

#### MapleSimSwerveDrivetrain (Robot Code)

**File**: `frc/robot/subsystems/simulation/MapleSimSwerveDrivetrain.java`

Bridges CTRE's swerve hardware API with MapleSim physics. Contains:

- **Constructor** - Builds the simulation configuration, creates physics bodies, registers with arena
- **`update()`** - Steps physics and injects gyro state
- **`SimSwerveModule`** (inner class) - Per-module motor controller simulation
- **`TalonFXMotorControllerSim`** - Feeds motor encoder state from physics, reads voltage commands
- **`TalonFXMotorControllerWithRemoteCanCoderSim`** - Same but also updates CANcoder sim state
- **`regulateModuleConstantsForSimulation()`** - Adjusts module constants for simulation compatibility:
  - Disables motor inversions (handled differently in sim)
  - Zeroes encoder offsets
  - Adjusts steer PID (kP=70, kD=4.5 for sim)
  - Reduces friction voltages

**Simulation parameters:**
| Parameter | Value |
|-----------|-------|
| Robot mass | 110 lbs (49.9 kg) |
| Bumper size | 30" x 30" |
| Drive motors | Kraken X60 FOC |
| Steer motors | Kraken X60 FOC |
| Wheel COF | 1.2 |
| Sim period | 0.005s (200Hz) |

#### MapSimSwerveTelemetry (Robot Code)

**File**: `frc/robot/subsystems/simulation/MapSimSwerveTelemetry.java`

Publishes simulation telemetry to NetworkTables for AdvantageScope visualization. Includes:
- Drive state (pose, speeds, module states/targets/positions)
- Field2d robot position
- Mechanism2d module visualizations (speed/direction per module)
- Game piece positions (`FieldSimulation/Fuel`)
- SignalLogger data for CTRE's signal logging

Only active in simulation (registered in `ControlBoard.tryInit()` when `Utils.isSimulation()` is true).

#### ElevatorWristSim (Robot Code)

**File**: `frc/robot/subsystems/simulation/ElevatorWristSim.java`

A **placeholder** Mechanism2d visualization for a future elevator + wrist mechanism. Currently:
- All dimension constants are 0 (not configured)
- Has a 3-stage elevator (S1, S2, S3) with a wrist at the top
- Publishes to SmartDashboard as `ElevatorWrist`
- Tracks state transitions with timestamps

This will be fleshed out when the physical elevator/wrist mechanism is designed.

### MapleSim Physics Engine (In-Tree)

The MapleSim source code is bundled under `org.ironmaple.simulation`. Key components:

#### SimulatedArena

**File**: `org/ironmaple/simulation/SimulatedArena.java`

The central physics world that manages all simulation objects. Uses the dyn4j physics engine for 2D rigid body dynamics.

Key methods:
- `simulationPeriodic()` - Steps the physics simulation
- `addDriveTrainSimulation()` - Registers a robot drivetrain
- `addGamePiece()` - Adds a game piece to the field
- `clearGamePieces()` - Removes all game pieces
- `getGamePiecesArrayByType(String)` - Gets positions of game pieces by type (for visualization)
- `overrideSimulationTimings()` - Sets simulation period and substeps

#### SwerveDriveSimulation

**File**: `org/ironmaple/simulation/drivesims/SwerveDriveSimulation.java`

Simulates the physics of a swerve drivetrain: forces from drive wheels, friction, collisions with field elements.

Key methods:
- `getSimulatedDriveTrainPose()` - Current robot pose in simulation
- `getDriveTrainSimulatedChassisSpeedsRobotRelative()` - Current velocity
- `setSimulationWorldPose()` - Teleport the robot to a position
- `getModules()` - Access individual module simulations

#### SwerveModuleSimulation

Each swerve module simulates:
- Drive motor torque → wheel force
- Steer motor torque → wheel angle
- Tire friction model (slip angle, wheel COF)
- Ground contact dynamics

#### IntakeSimulation

**File**: `org/ironmaple/simulation/IntakeSimulation.java`

Simulates game piece collection through contact detection:
- Configured as "over the bumper" intake on the FRONT side
- 10" x 10" contact zone
- Can hold up to 30 game pieces
- `startIntake()` / `stopIntake()` to enable/disable collection

#### Season-Specific: 2026 REBUILT

**File**: `org/ironmaple/simulation/seasonspecific/rebuilt2026/`

- `Arena2026Rebuilt.java` - Field layout with obstacles, hubs, outposts
- `RebuiltFuelOnField.java` - Fuel game piece physics (on the ground)
- `RebuiltFuelOnFly.java` - Fuel game piece in the air (projectile)
- `RebuiltHub.java` - Hub scoring target
- `RebuiltOutpost.java` - Human player station

#### Motor Simulation

**File**: `org/ironmaple/simulation/motorsims/`

- `MapleMotorSim.java` - DC motor physics simulation
- `SimMotorConfigs.java` - Motor characteristic configurations
- `SimMotorState.java` - Motor state (angle, velocity)
- `SimulatedBattery.java` - Battery voltage model (voltage droop under load)
- `SimulatedMotorController.java` - Interface for motor controller simulation

#### COTS Configurations

**File**: `org/ironmaple/simulation/drivesims/COTS.java`

Pre-configured "Commercial Off-The-Shelf" hardware models:
- `ofPigeon2()` - Pigeon 2 gyro simulation configuration

#### Utility Classes

**File**: `org/ironmaple/utils/`

- `FieldMirroringUtils.java` - Coordinate mirroring for Red/Blue alliance
- `GeometryConvertor.java` - Converts between WPILib and dyn4j geometry types
- `MapleCommonMath.java` - Interpolation, clamping, and other math utilities
- `SwerveStateProjection.java` - Projects swerve module states for visualization

## Simulation Tips

### Common Issues

1. **Robot doesn't move in sim**: Make sure you're in "Teleoperated" or "Autonomous" mode in the Sim GUI
2. **Robot spins wildly**: Steer PID might need simulation-specific tuning (handled by `regulateModuleConstantsForSimulation()`)
3. **No game pieces visible**: Check AdvantageScope's 3D field tab and ensure `FieldSimulation/Fuel` is being published
4. **Incorrect starting position**: The simulated robot starts at (3, 3) by default. Use `resetPose()` to change it.

### Limitations

- Simulation is 2D only (no vertical dynamics for climbing, elevator, etc.)
- Game piece scoring detection is simplified
- Motor models are approximations of real motor behavior
- CAN bus latency is not simulated
- Vision simulation is not implemented (Limelight returns null poses in sim)

## External Resources

- [MapleSim Documentation](https://shenzhen-robotics-alliance.github.io/maple-sim/)
- [MapleSim GitHub](https://github.com/Shenzhen-Robotics-Alliance/maple-sim)
- [dyn4j Physics Engine](https://dyn4j.org/)
- [dyn4j Getting Started](https://dyn4j.org/pages/getting-started.html)
- [AdvantageScope Documentation](https://docs.advantagescope.org/)
- [AdvantageScope 3D Field](https://docs.advantagescope.org/tab-reference/3d-field/)
- [WPILib Simulation Guide](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/introduction.html)
