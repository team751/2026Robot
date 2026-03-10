# State Machine Audit

Comprehensive review of state machine logic across all subsystems.

**Severity legend:**
- 🔴 CRITICAL — Safety risk or motor commanded incorrectly regardless of state
- 🟠 HIGH — Significant functional bug, subsystem won't work as intended
- 🟡 MEDIUM — Logic issue that causes unexpected behavior in specific scenarios
- 🟢 LOW — Code quality / maintenance concern, no immediate runtime impact

---

## ExtenderSubsystem

### 1. ✅ DONE: Operator precedence bug (lines 62, 65)

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

### 2. ✅ DONE: Limit switch checks only run on state transition (lines 62–75)

All limit switch logic is inside `if (nextState != state)`. Limit switches are only evaluated at the **instant** of a state transition, not continuously while the motor is running. If the extender starts and then hits a limit switch on the next cycle, it won't be detected.

**Fix:** Move limit switch checks **outside** the `if (nextState != state)` block.

### 3. ✅ DONE: `requestExtending()` doesn't clear `requestedIdle` (lines 90–93)

```java
public void requestExtending() {
    requestedRetracting = false;
    requestedExtending = true;
}
```

If `requestIdle()` was called (sets `requestedIdle = true`) and then `requestExtending()` is called before `periodic()` runs, `requestedIdle` is still true. Since `periodic()` checks `requestedIdle` first, idle wins and extending is ignored. Note: `requestRetracting()` does call `unsetAllRequests()`, so only `requestExtending()` has this inconsistency.

### 4. ✅ DONE: `requestIdle()` bypasses the state machine (line 103)

```java
public void requestIdle() {
    unsetAllRequests();
    requestedIdle = true;
    setExtenderMotor(0);  // directly commands motor outside periodic()
}
```

This stops the motor immediately, outside the state machine in `periodic()`. Can cause a mismatch where the motor is stopped but `state` still says EXTENDING/RETRACTING until the next cycle.

### 5. 🔴 CRITICAL: Limit switch polarity likely inverted (lines 62, 65, 68, 72)

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

### 8. ✅ DONE: `requestIdle()` permanently zeroes the shared speed constants (line 84)

`newSpeed()` was removed entirely and `requestIdle()` no longer calls it — it just clears request flags. Constants are no longer mutated on idle.

### 9. ✅ DONE: `newSpeed()` while already SPINNING doesn't update motors (line 72)

`newSpeed()` method was removed entirely. Speed is now read directly from `ShooterConstants` at transition time, so this race condition no longer exists.

---

## TransferSubsystem

### 10. 🟡 MEDIUM: `requestTransferring()` and `requestReversing()` don't clear `requestedIdle` (lines 72–79)

Same pattern as Intake/Extender. If `requestIdle()` was called and then `requestTransferring()` follows before `periodic()`, idle wins.

---

## Superstructure

### 11. 🟠 HIGH: Request flags are never read — state machine is frozen (lines 51–55)

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

### 12. 🟡 MEDIUM: Inconsistent request method patterns across all subsystems

`requestIdle()` in every subsystem calls `unsetAllRequests()` before setting its flag. But non-idle request methods (e.g., `requestIntaking()`, `requestExtending()`, `requestTransferring()`) only selectively clear the opposing request and **don't** clear `requestedIdle`. This means idle always wins in a race condition. Either all request methods should call `unsetAllRequests()`, or the selective clearing should also include `requestedIdle`.
