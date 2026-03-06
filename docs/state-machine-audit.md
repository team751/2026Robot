# State Machine Audit

Comprehensive review of state machine logic across all subsystems.

---

## ExtenderSubsystem

### 1. CRITICAL: Operator precedence bug (lines 62, 65)

```java
if (state == ExtenderState.EXTENDING && (LeftLimit.get()) || (RightLimit.get())) {
```

`&&` binds tighter than `||`, so this parses as:

```java
if ((state == ExtenderState.EXTENDING && LeftLimit.get()) || RightLimit.get()) {
```

If `RightLimit.get()` is true, the motor gets set to 0.5V **regardless of state** — even during IDLE or RETRACTING. Same bug on line 65 for RETRACTING. Fix: wrap the `||` in parentheses:

```java
if (state == ExtenderState.EXTENDING && (LeftLimit.get() || RightLimit.get())) {
```

### 2. CRITICAL: Limit switch checks only run on state transition (lines 62–75)

All limit switch logic is inside `if (nextState != state)`. Limit switches are only evaluated at the **instant** of a state transition, not continuously while the motor is running. If the extender starts and then hits a limit switch on the next cycle, it won't be detected.

**Fix:** Move limit switch checks **outside** the `if (nextState != state)` block.

### 3. `requestExtending()` doesn't clear `requestedIdle` (lines 90–93)

```java
public void requestExtending() {
    requestedRetracting = false;
    requestedExtending = true;
}
```

If `requestIdle()` was called (sets `requestedIdle = true`) and then `requestExtending()` is called before `periodic()` runs, `requestedIdle` is still true. Since `periodic()` checks `requestedIdle` first, idle wins and extending is ignored. Note: `requestRetracting()` does call `unsetAllRequests()`, so only `requestExtending()` has this inconsistency.

### 4. `requestIdle()` bypasses the state machine (line 103)

```java
public void requestIdle() {
    unsetAllRequests();
    requestedIdle = true;
    setExtenderMotor(0);  // directly commands motor outside periodic()
}
```

This stops the motor immediately, outside the state machine in `periodic()`. Can cause a mismatch where the motor is stopped but `state` still says EXTENDING/RETRACTING until the next cycle.

### 5. Limit switch polarity likely inverted (lines 62, 65, 68, 72)

`DigitalInput.get()` returns `true` when the circuit is **open** (switch NOT pressed) for typical normally-closed FRC limit switches. The code treats `get() == true` as "limit hit", which means the slow-down/stop logic fires when switches are **not** pressed. Double-check wiring — you may need `!LeftLimit.get()`.

---

## IntakeSubsystem

### 6. ✅ DONE: `spitSpeed` is positive — same direction as intake (IntakeConstants.java:14)

```java
public static final double intakeSpeed = 3.5;
public static final double spitSpeed = 3.5;
```

Both are positive. The motor is configured `CounterClockwise_Positive` with the comment `// Positive intake`. So SPITTING runs the motor at +3.5V — the **same direction and speed** as INTAKING. For spitting to reverse the motor, `spitSpeed` should be negative (e.g., `-3.5`).

### 7. ✅ DONE: `requestIntaking()` and `requestSpitting()` don't clear `requestedIdle` (lines 69–82)

Neither method clears `requestedIdle`, so if `requestIdle()` was called in the same cycle, idle always wins since it's checked first in `periodic()`.

---

## ShooterSubsystem

### 8. `requestIdle()` permanently zeroes the shared speed constants (line 84)

```java
public void requestIdle() {
    newSpeed(0, 0);  // sets ShooterConstants.flywheelSpeed = 0, ShooterConstants.backSpeed = 0
    unsetAllRequests();
    requestedIdle = true;
}
```

`newSpeed(0, 0)` writes `0` into `ShooterConstants.flywheelSpeed` and `ShooterConstants.backSpeed`. After calling `requestIdle()`, any subsequent `requestShoot()` (without `newSpeed`) will transition to SPINNING with 0 voltage — the shooter won't spin.

### 9. `newSpeed()` while already SPINNING doesn't update motors (line 72)

```java
public void newSpeed(double flySpeed, double backSpeed) {
    ShooterConstants.flywheelSpeed = flySpeed;
    ShooterConstants.backSpeed = backSpeed;
    this.requestShoot();
}
```

If already in SPINNING, `nextState == state` so the transition block never runs. The new speeds are stored in constants but **never applied** to the motors until the next full state transition (e.g., idle -> spinning).

---

## ClimberSubsystem

### 10. Limit switches declared but never used (lines 22–23)

`LimitL` and `LimitR` are created on DIO 7 and 8 but never referenced in any logic. The climber has no limit-switch safety stops.

### 11. `spinSlow()` only drives the left motor (lines 83–85)

```java
public void spinSlow(int direction) {
    leftClimber.setControl(new DutyCycleOut(direction * 0.1));
}
```

The right motor is not controlled — it holds whatever command it had before, or is undriven.

### 12. `StrictFollower(10)` hardcoded CAN ID (lines 100, 106)

```java
rightClimber.setControl(new StrictFollower(10));
```

The `10` should reference the left climber's CAN ID from constants. If the CAN ID changes, this will silently follow the wrong device.

### 13. `moveUp180()`/`moveDown180()` will do nothing — all PID gains are zero (ClimberConstants.java:35–42)

`PositionVoltage` uses Slot0 PID, but all gains (kP, kI, kD, kS, kV, kG, kA) are `0.0`. With kP=0, the position controller outputs 0 voltage regardless of error. These methods are dead code until gains are tuned.

### 14. `moveUp180()`/`moveDown180()` compute target from left motor only (lines 123, 131)

Both methods read `leftClimber.getPosition()` to calculate `targetPosition`, then apply that same target to both motors. If the motors have drifted to different positions, the right motor gets a wrong target.

### 15. Error averaging formula is mathematically wrong (line 116)

```java
ClimberConstants.averageMotorError = (ClimberConstants.averageMotorError + error) / 3;
```

This isn't a running average — it converges toward 0 regardless of actual error. For a proper exponential moving average: `alpha * error + (1 - alpha) * averageMotorError`.

### 16. `stopSpinUntil()` then `spinning = false` is redundant (lines 67–68)

In `periodic()`, `stopSpinUntil()` already sets `spinning = false` on line 112. Line 68 sets it again. Not harmful, just redundant.

---

## TransferSubsystem

### 17. `requestTransferring()` and `requestReversing()` don't clear `requestedIdle` (lines 72–79)

Same pattern as Intake/Extender. If `requestIdle()` was called and then `requestTransferring()` follows before `periodic()`, idle wins.

---

## Superstructure

### 18. Request flags are never read — state machine is frozen (lines 51–55)

```java
switch (systemState) {
    case PRE_HOME -> {}
    case IDLE -> {}
    default -> throw new IllegalArgumentException("wops");
}
```

`requestHome` and `requestIdle` are set by request methods but never checked in `periodic()`. `nextState` always equals `systemState`, so no transition ever occurs. The superstructure is permanently stuck in `PRE_HOME`. (May be intentional WIP.)

---

## Cross-Cutting Issue

### 19. Inconsistent request method patterns across all subsystems

`requestIdle()` in every subsystem calls `unsetAllRequests()` before setting its flag. But non-idle request methods (e.g., `requestIntaking()`, `requestExtending()`, `requestTransferring()`) only selectively clear the opposing request and **don't** clear `requestedIdle`. This means idle always wins in a race condition. Either all request methods should call `unsetAllRequests()`, or the selective clearing should also include `requestedIdle`.
