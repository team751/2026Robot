# Utilities

This document covers the reusable library classes in `frc.lib` and the utility classes in `frc.robot.util`.

## Library Classes (frc.lib)

These are general-purpose utilities that could be reused across FRC projects.

---

### TunableParameter

**File**: `frc/lib/TunableParameter.java`

Enables live-tuning of numeric values via SmartDashboard without redeploying code. Each `TunableParameter` publishes a number to SmartDashboard and monitors it for changes. When the value changes on the dashboard, a callback fires.

#### How It Works

```java
// 1. Create a tunable parameter (auto-registers itself)
new TunableParameter("Shooter/Speed", 12.0, (value) -> {
    this.shooterVoltage = value;
});

// 2. Every robot periodic cycle, poll for changes
TunableParameter.updateAll();  // Called in Robot.robotPeriodic()
```

#### Internal Design

- All `TunableParameter` instances auto-register into a static `ArrayList<TunableParameter>`
- `updateAll()` iterates through every registered parameter and calls `fetch()`
- `fetch()` reads the current SmartDashboard value and compares to the cached value
- Only fires the callback when the value actually changes (uses `!=` comparison)
- Initial value is pushed to SmartDashboard in the constructor

#### API

| Method / Constructor | Description |
|---------------------|-------------|
| `TunableParameter(String name, double initialValue, DoubleConsumer callback)` | Creates a tunable parameter, publishes to SmartDashboard, auto-registers |
| `static updateAll()` | Polls all registered parameters for changes. Call every periodic cycle. |

#### Usage Examples

```java
// PID tuning
new TunableParameter("Drive/kP", 0.5, (v) -> pidController.setP(v));
new TunableParameter("Drive/kD", 0.01, (v) -> pidController.setD(v));

// Speed adjustment
new TunableParameter("Intake/Voltage", 8.0, (v) -> intakeVoltage = v);
```

#### Best Practices

- Always use descriptive names with category prefixes (e.g., `"Shooter/Speed"`)
- Remove or replace with constants before competition (SmartDashboard polling has overhead)
- The callback is NOT called on construction — only when the value changes at runtime

---

### PS5Controller

**File**: `frc/lib/PS5Controller.java`

A custom wrapper around WPILib's `Joystick` class that provides named fields for all PS5 DualSense controller inputs with proper axis inversion.

#### Available Inputs

**Joystick Axes** (all inverted so up/left = positive):
| Field | Type | Description |
|-------|------|-------------|
| `leftVerticalJoystick` | `DoubleSupplier` | Left stick Y (up = positive) |
| `leftHorizontalJoystick` | `DoubleSupplier` | Left stick X (left = positive) |
| `rightVerticalJoystick` | `DoubleSupplier` | Right stick Y (up = positive) |
| `rightHorizontalJoystick` | `DoubleSupplier` | Right stick X (left = positive) |

**Triggers:**
| Field | Type | Description |
|-------|------|-------------|
| `leftTrigger` | `JoystickButton` | L2 trigger |
| `rightTrigger` | `JoystickButton` | R2 trigger |

**Bumpers:**
| Field | Type | Description |
|-------|------|-------------|
| `leftBumper` | `JoystickButton` | L1 bumper |
| `rightBumper` | `JoystickButton` | R1 bumper |

**Face Buttons:**
| Field | Type | Description |
|-------|------|-------------|
| `triangleButton` | `JoystickButton` | Triangle (top) |
| `circleButton` | `JoystickButton` | Circle (right) |
| `squareButton` | `JoystickButton` | Square (left) |
| `crossButton` | `JoystickButton` | Cross/X (bottom) |

**D-Pad** (using POV hat):
| Field | Type | POV Angle |
|-------|------|-----------|
| `dUp` | `Trigger` | 0 degrees |
| `dRight` | `Trigger` | 90 degrees |
| `dDown` | `Trigger` | 180 degrees |
| `dLeft` | `Trigger` | 270 degrees |

**Other:**
| Field | Type | Description |
|-------|------|-------------|
| `touchpadButton` | `JoystickButton` | Touchpad press |
| `leftJoystickButton` | `JoystickButton` | L3 (left stick press) |
| `rightJoystickButton` | `JoystickButton` | R3 (right stick press) |

#### Usage in ControlBoard

```java
PS5Controller driver = new PS5Controller(0);  // Port 0

// Button bindings
driver.circleButton.onTrue(new InstantCommand(() -> drive.bigResetPose()));
driver.rightBumper.whileTrue(new StartEndCommand(
    () -> preciseControl = true,
    () -> preciseControl = false
));

// Reading axes
double x = driver.leftVerticalJoystick.getAsDouble();
double y = driver.leftHorizontalJoystick.getAsDouble();
double rot = driver.rightHorizontalJoystick.getAsDouble();
```

#### Axis Inversion

All axes are negated (`-joystick.getRawAxis(...)`) because:
- WPILib's raw axis convention: pushing up returns negative, pushing left returns negative
- FRC convention (and this code): up/left should be positive
- This matches WPILib field coordinates: +X = forward, +Y = left

---

### CTREConfig

**File**: `frc/lib/CTREConfig.java`

A generic builder pattern for configuring and creating CTRE Phoenix 6 devices. Simplifies the boilerplate of creating a motor controller with a specific CAN ID, bus, and configuration.

#### Builder Pattern

```java
// Define configuration
CTREConfig<TalonFX, TalonFXConfiguration> motorConfig =
    new CTREConfig<>(TalonFXConfiguration::new)
        .withName("Shooter Motor")
        .withCanID(15)
        .withBus(Robot.drivebus)
        .withOptimizeBus(true);

// Configure motor settings
TalonFXConfiguration config = motorConfig.config;
config.Slot0.kP = 0.5;
config.CurrentLimits.StatorCurrentLimit = 40;

// Create the device
TalonFX motor = motorConfig.createDevice(TalonFX::new);
```

#### What `createDevice()` Does

1. Creates the device using the supplied constructor (`TalonFX::new`)
2. Checks if the device is connected (logs error if not)
3. Applies the configuration via `CTREUtil.applyConfiguration()`
4. If `optimizeBus` is true AND the device is a TalonFX:
   - Sets position, velocity, voltage, stator current, supply current signals to 50Hz update rate
   - Calls `optimizeBusUtilization()` to reduce CAN traffic

#### Fields

| Field | Type | Default | Description |
|-------|------|---------|-------------|
| `name` | `String` | `"UNNAMED"` | Human-readable device name |
| `canID` | `int` | `0` | CAN bus ID |
| `canbus` | `CANBus` | `Robot.riobus` | Which CAN bus the device is on |
| `config` | `Config` | Supplier result | Device-specific configuration object |
| `optimizeBus` | `boolean` | `true` | Whether to optimize CAN bus utilization |

#### DeviceSupplier Interface

```java
public interface DeviceSupplier<Device> {
    Device get(int canID, CANBus canbus);
}
```

Usage: `motorConfig.createDevice(TalonFX::new)` — TalonFX's constructor matches this signature.

---

### CTREUtil

**File**: `frc/lib/CTREUtil.java`

Static utility methods for CTRE Phoenix 6 error handling. CTRE API calls return `StatusCode` values that can indicate transient failures — these utilities retry operations to improve reliability.

#### Methods

| Method | Description |
|--------|-------------|
| `tryUntilOK(Supplier<StatusCode>, int deviceId)` | Retries a CTRE operation up to 10 times until it returns `StatusCode.OK`. Reports error via DriverStation if all retries fail. |
| `applyConfiguration(TalonFX, TalonFXConfiguration)` | Applies a TalonFX configuration with retry logic |
| `applyConfiguration(CANrange, CANrangeConfiguration)` | Applies a CANrange configuration with retry logic |
| `applyConfiguration(CANcoder, CANcoderConfiguration)` | Applies a CANcoder configuration with retry logic |
| `applyConfiguration(CANdle, CANdleConfiguration)` | Applies a CANdle configuration with retry logic |
| `applyConfiguration(ParentDevice, ParentConfiguration)` | Generic version that dispatches to the correct typed method |
| `refreshConfiguration(TalonFX, TalonFXConfiguration)` | Reads current motor configuration with retry logic |

#### Error Handling

```java
// Retries up to 10 times
public static StatusCode tryUntilOK(Supplier<StatusCode> function, int deviceId) {
    for (int i = 0; i < 10; i++) {
        StatusCode code = function.get();
        if (code == StatusCode.OK) return code;
    }
    DriverStation.reportError("Error on CTRE device " + deviceId + ": " + statusCode, true);
    return statusCode;
}
```

#### Why This Exists

CTRE Phoenix 6 API calls communicate over CAN bus, which can experience transient failures (bus congestion, device booting up, etc.). Without retry logic, a single CAN frame drop during initialization could leave a motor unconfigured. These utilities ensure reliable device setup.

---

## Robot Utility Classes (frc.robot.util)

### ControlBoard

**File**: `frc/robot/util/ControlBoard.java`

The central hub for all controller input and command bindings. Manages two PS5 controllers (driver and operator) and creates the default drive command.

#### Lazy Initialization

The `ControlBoard` uses lazy initialization for controllers:
1. Constructor calls `tryInit()` immediately
2. `Robot.driverStationConnected()` calls `tryInit()` again
3. `tryInit()` only creates controllers if they're `null`

This handles the case where the Driver Station hasn't connected yet when the robot boots.

#### Controller Ports

| Role | Port | PS5Controller Field |
|------|------|-------------------|
| Driver | 0 | `driver` |
| Operator | 1 | `operator` |

#### Driver Bindings

| Button | Action |
|--------|--------|
| Left stick | Translation (forward/backward, left/right) |
| Right stick X | Rotation |
| Right bumper (hold) | Precise control mode (25% speed, 50% rotation) |
| Circle | Reset odometry to (0,0,0) |

#### Drive Request Configuration

```java
SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric()
    .withDeadband(maxSpeed * 0.05)                    // 5% translation deadband
    .withRotationalDeadband(maxAngularSpeed * 0.1)    // 10% rotation deadband
    .withDriveRequestType(OpenLoopVoltage)             // Open-loop drive
    .withSteerRequestType(Position)                    // Closed-loop steer
    .withDesaturateWheelSpeeds(true)                   // Prevent wheel speed saturation
    .withForwardPerspective(OperatorPerspective);      // Alliance-aware
```

#### Speed Scaling

```java
// Base: 60% translation, 80% rotation
velocityX = 0.6 * maxSpeed * x * scale
velocityY = 0.6 * maxSpeed * y * scale
rotationalRate = 0.8 * maxAngularSpeed * rot^2 * rotScale  // Squared for sensitivity

// Precise mode: 25% translation, 50% rotation
scale = preciseControl ? 0.25 : 1.0
rotScale = preciseControl ? 0.50 : 1.0
```

#### Simulation Telemetry

In simulation, `MapSimSwerveTelemetry` is registered as a telemetry callback:
```java
if (Utils.isSimulation())
    drive.registerTelemetry(new MapSimSwerveTelemetry(maxSpeed)::telemeterize);
```

---

### FieldConstants

**File**: `frc/robot/util/FieldConstants.java`

Defines all field geometry for the 2026 REBUILT game, derived from AprilTag positions.

See [Field Constants Documentation](field-constants.md) for full details.

---

### Constants (Deprecated)

**File**: `frc/robot/util/Constants.java`

Contains the deprecated `drivebus` string constant. Use `Robot.drivebus` instead.

```java
@Deprecated
public static final String drivebus = "Drivebus";

public static final boolean disableHAL = !HAL.initialize(500, 0);
```

`disableHAL` is used by `FieldConstants` to determine file paths for AprilTag layouts (deploy directory vs. source directory).

---

### LimelightHelpers

**File**: `frc/robot/util/LimelightHelpers.java`

The official Limelight helper library (v1.14). This is a large file provided by Limelight that wraps NetworkTables communication.

See [Vision & Odometry Documentation](vision-and-odometry.md) for usage details.
