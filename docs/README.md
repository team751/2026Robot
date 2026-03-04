# Team 751 - 2026 Robot Documentation

Welcome to the documentation for FRC Team 751's 2026 REBUILT season robot code.

## Documentation Index

| Document | Description |
|----------|-------------|
| [Getting Started](getting-started.md) | Setup guide for new developers: tools, environment, building, deploying |
| [Architecture Overview](architecture.md) | High-level system design, control flow, and design patterns |
| [Swerve Drive](swerve-drive.md) | Swerve drivetrain: hardware, CTRE Phoenix 6 API, tuning, operator controls |
| [Vision & Odometry](vision-and-odometry.md) | Limelight cameras, AprilTag localization, pose estimation, sensor fusion |
| [Autonomous](autonomous.md) | PathPlanner setup, auto routines, path creation, trajectory following |
| [Simulation](simulation.md) | MapleSim physics simulation, AdvantageScope visualization, running in sim |
| [Subsystems Reference](subsystems.md) | Detailed reference for every subsystem: Superstructure, Shooter, etc. |
| [Utilities](utilities.md) | Library classes: TunableParameter, PS5Controller, CTREConfig, CTREUtil |
| [Field Constants](field-constants.md) | 2026 REBUILT field geometry, AprilTag layout, scoring zones |
| [External Resources](external-resources.md) | Links to all vendor docs, APIs, game manual, and learning resources |

## Quick Reference

```bash
# Build
./gradlew build

# Deploy to robot
./gradlew deploy

# Run simulation
./gradlew simulateJava

# Format code
./gradlew spotlessApply
```

**Team Number:** 751
**Game:** 2026 REBUILT
**Language:** Java 17
**Framework:** WPILib Command-Based + CTRE Phoenix 6 Swerve

## Project Structure

```
src/main/java/
  frc/
    lib/                          # Reusable utility classes
      CTREConfig.java             # CTRE device configuration builder
      CTREUtil.java               # CTRE retry/error handling
      PS5Controller.java          # PS5 DualSense controller wrapper
      TunableParameter.java       # Live SmartDashboard tuning
    robot/
      Constants.java              # Global constants (mostly unused)
      Main.java                   # Entry point (do not modify)
      Robot.java                  # TimedRobot lifecycle, CAN bus definitions
      subsystems/
        Superstructure.java       # Central state machine coordinator
        drive/
          Odometry.java           # Vision + wheel odometry fusion
          SwerveConstants.java    # Drive speed limits, PathPlanner config
          SwerveSubsystem.java    # Swerve drive subsystem (singleton)
          generated/
            TunerConstants.java   # Tuner X generated swerve config
        shooter/
          ShooterConstants.java   # Shooter motor config (commented out)
          ShooterSubsystem.java   # Shooter state machine (commented out)
        simulation/
          ElevatorWristSim.java   # Mechanism2d elevator/wrist viz
          MapleSimSwerveDrivetrain.java  # CTRE↔MapleSim bridge
          MapSimSwerveTelemetry.java     # Sim telemetry publisher
        vision/
          LimelightConstants.java # Camera IPs, offsets, names
          LimelightSubsystem.java # Dual Limelight AprilTag subsystem
      util/
        Constants.java            # Deprecated CAN bus strings
        ControlBoard.java         # Controller bindings, drive request
        FieldConstants.java       # 2026 field geometry from AprilTags
        LimelightHelpers.java     # Limelight NetworkTables helper (v1.14)
  org/ironmaple/                  # MapleSim physics sim (bundled in-tree)
    simulation/
      SimulatedArena.java         # Central physics world
      IntakeSimulation.java       # Game piece intake sim
      Goal.java                   # Scoring target simulation
      drivesims/                  # Drive train simulation
      gamepieces/                 # Game piece physics
      motorsims/                  # Motor simulation
      seasonspecific/rebuilt2026/  # 2026 field elements
    utils/                        # Math and field mirroring utilities

src/main/deploy/
  pathplanner/                    # PathPlanner auto routines and paths
    autos/                        # Auto routine definitions (.auto)
    paths/                        # Path definitions (.path)
    settings.json                 # PathPlanner robot configuration
    navgrid.json                  # Navigation grid for pathfinding

vendordeps/                       # Vendor dependency JSON files
```
