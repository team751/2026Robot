# Getting Started

This guide walks you through setting up your development environment and getting the robot code running.

## Prerequisites

### Required Software

1. **WPILib 2026** (includes VS Code, Java 17 JDK, and Gradle)
   - Download from [WPILib Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
   - This installs everything you need: VS Code with WPILib extensions, Java 17 JDK, Gradle, and the WPILib libraries

2. **Git** for version control
   - macOS: `xcode-select --install` or [git-scm.com](https://git-scm.com/)
   - Windows: [Git for Windows](https://gitforwindows.org/)

3. **CTRE Phoenix Tuner X** (for motor controller configuration)
   - Download from the [Microsoft Store](https://apps.microsoft.com/store/detail/phoenix-tuner-x/9NVV4PWDW27Z) or [CTRE Downloads](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/index.html)
   - Required to configure motor IDs, tune PIDs, and generate swerve constants

4. **PathPlanner** (for autonomous path design)
   - Download from [pathplanner.dev](https://pathplanner.dev/) or the Microsoft Store
   - GUI tool for designing autonomous paths and routines

### Optional but Recommended

- **AdvantageScope** - Data visualization and replay tool ([docs.advantagescope.org](https://docs.advantagescope.org/))
- **Elastic Dashboard** - Alternative to SmartDashboard/Shuffleboard
- **Limelight Web Interface** - Access at `http://10.7.51.71:5801` (front) or `http://10.7.51.75:5801` (back)

## Cloning the Repository

```bash
git clone <repository-url>
cd 2026Robot
```

## Building the Code

The project uses Gradle via the GradleRIO plugin. You do NOT need to install Gradle separately.

```bash
# Build the project (compiles, checks for errors)
./gradlew build

# On Windows, use:
gradlew.bat build
```

The first build will take several minutes to download dependencies. Subsequent builds are much faster.

### Common Build Commands

```bash
# Full build with formatting check
./gradlew build

# Build without running tests
./gradlew assemble

# Format all code (Google Java Format via Spotless)
./gradlew spotlessApply

# Check if code is formatted correctly (CI will fail if not)
./gradlew spotlessCheck

# Clean build artifacts
./gradlew clean
```

## Deploying to the Robot

### Connection Methods

1. **USB**: Connect a USB-A to USB-B cable from your laptop to the RoboRIO
2. **Wi-Fi**: Connect to the robot's radio (`751` network)
3. **Ethernet**: Direct ethernet to the robot's radio

### Deploy Command

```bash
# Deploy robot code to the RoboRIO
./gradlew deploy
```

This compiles the code, creates a JAR, and uploads it to the RoboRIO at `/home/lvuser/`. The robot code starts automatically after deploy.

### Troubleshooting Deploy

- **Can't find roboRIO**: Make sure you're connected via USB/Wi-Fi/Ethernet. The RoboRIO should be at `10.7.51.2`.
- **Build fails**: Run `./gradlew build` first to see compilation errors
- **Old code running**: Try `./gradlew deploy --offline` or restart the robot

## Running Simulation

Simulation lets you test your code without a physical robot.

```bash
# Launch simulation with GUI
./gradlew simulateJava
```

This opens:
1. **Simulation GUI** - Shows robot state, lets you enable/disable modes
2. **Simulated Driver Station** - Acts as the FRC Driver Station

### Using Simulation

1. In the Simulation GUI, drag "Autonomous" or "Teleop" to the robot state
2. Open **AdvantageScope** and connect to `localhost` (NetworkTables) to visualize the robot
3. The MapleSim physics engine simulates swerve drive physics, game pieces, and field collisions
4. Use a gamepad connected to your computer to drive in teleop

### Simulation Architecture

The simulation uses **MapleSim** (powered by the dyn4j physics engine) running at 200Hz on a separate thread. It simulates:
- Swerve module physics (drive motors, steer motors, wheel friction)
- Robot collision body (30"x30" bumpers, 110 lbs)
- Game piece interactions (fuel pickup via intake simulation)
- Field obstacles (hubs, outposts, barriers)

See [Simulation Documentation](simulation.md) for details.

## Project Organization

### Package Structure

- `frc.robot` - Main robot code (Robot, Main, Constants)
- `frc.robot.subsystems` - All subsystems (Superstructure, drive, vision, etc.)
- `frc.robot.subsystems.drive` - Swerve drive code
- `frc.robot.subsystems.vision` - Limelight vision processing
- `frc.robot.subsystems.simulation` - Simulation-only code
- `frc.robot.subsystems.shooter` - Shooter subsystem (currently commented out)
- `frc.robot.util` - Utility classes (ControlBoard, FieldConstants, LimelightHelpers)
- `frc.lib` - Reusable library classes (PS5Controller, TunableParameter, CTRE utilities)
- `org.ironmaple.simulation` - MapleSim physics engine (bundled in-tree)

### Key Design Patterns

1. **Singleton Pattern** - All subsystems use `getInstance()`. Never call `new SubsystemName()`.
2. **Command-Based** - Uses WPILib's command-based framework. Subsystems define capabilities, commands compose behavior.
3. **Request-Based State Machines** - Subsystems like Superstructure use `requestState()` methods to transition between states.
4. **Separation of Constants** - Each subsystem has its own constants file (e.g., `SwerveConstants.java`, `LimelightConstants.java`).

## Common Development Tasks

### Adding a New Subsystem

1. Create `NewSubsystem.java` in the appropriate package under `frc.robot.subsystems`
2. Extend `SubsystemBase`
3. Add the singleton pattern:
   ```java
   private static NewSubsystem instance;
   public static NewSubsystem getInstance() {
       if (instance == null) instance = new NewSubsystem();
       return instance;
   }
   private NewSubsystem() { /* constructor */ }
   ```
4. Initialize in `Robot.java` constructor: `NewSubsystem.getInstance();`
5. Create a constants file if needed (e.g., `NewSubsystemConstants.java`)

### Adding a Controller Binding

1. Open `ControlBoard.java`
2. Add bindings in `configureDriverBindings()` or `configureOperatorBindings()`
3. Use the `PS5Controller` fields: `controller.crossButton`, `controller.leftTrigger`, etc.
4. Example:
   ```java
   controller.crossButton.onTrue(new InstantCommand(() -> subsystem.requestAction()));
   ```

### Adding a PathPlanner Auto

1. Open PathPlanner GUI
2. Create paths in the Paths tab
3. Compose paths into autos in the Autos tab
4. Save - files are stored in `src/main/deploy/pathplanner/`
5. The auto chooser in SmartDashboard automatically picks up new autos

### Tuning a Value at Runtime

Use `TunableParameter` to adjust values from SmartDashboard without redeploying:

```java
new TunableParameter("Shooter/Speed", 12.0, (value) -> {
    // Called whenever the value changes in SmartDashboard
    this.shooterSpeed = value;
});
```

## Getting Help

- **WPILib Docs**: https://docs.wpilib.org/en/stable/
- **CTRE Phoenix 6 Docs**: https://v6.docs.ctr-electronics.com/en/stable/
- **Chief Delphi** (FRC community forum): https://www.chiefdelphi.com/
- **FRC Discord**: https://discord.gg/frc
