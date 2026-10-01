# Vision & Odometry

This document covers the PhotonVision AprilTag cameras, how their pose estimates are filtered and fused with wheel odometry, how to debug them in AdvantageScope, and how to bring up and validate the cameras on a real robot.

> **Status:** The Limelights have been removed from the code. The two PhotonVision cameras have only been tested in sim so far. The camera transforms in `PhotonVisionConstants` are **placeholders** (`TODO(measure)`). Go through the [bring-up checklist](#coprocessor-bring-up-checklist) and the [on-robot validation](#on-robot-validation) before trusting vision in a match.

## How the Robot Knows Where It Is

### 1. Wheel Odometry (Dead Reckoning)

CTRE's `SwerveDrivetrain` (inside `SwerveSubsystem`) integrates wheel encoder and Pigeon 2 readings on its own high-rate odometry thread (250Hz on CAN FD).

**Pros**: Always available, high update rate, smooth
**Cons**: Drifts over time from wheel slip, collisions, and encoder error

### 2. Vision (AprilTag Localization)

Two cameras plugged into a PhotonVision coprocessor detect [AprilTags](https://docs.wpilib.org/en/stable/docs/software/vision-processing/apriltag/apriltag-intro.html). Every tag has a known pose on the field, so a camera that sees tags can work out where the robot is.

**Pros**: Absolute position (no drift), corrects accumulated wheel odometry error
**Cons**: Only works when tags are visible, lower update rate, and a single far or ambiguous tag can be confidently wrong

### 3. Sensor Fusion

`Odometry.java` filters each vision estimate and passes the survivors to `SwerveSubsystem.addVisionMeasurement()` (CTRE's latency-compensated pose estimator), along with a standard deviation that says how much to trust it.

**Important**: Only the **translation** (x, y) from vision is fused. The **heading** always comes from the Pigeon 2. The fused `Pose2d` uses the gyro's rotation, and the theta std dev is `99999`.

## Hardware

| | Front camera | Side camera |
|---|---|---|
| PV camera nickname | `front` (`FRONT_CAMERA_NAME`) | `side` (`SIDE_CAMERA_NAME`) |
| Robot-to-camera transform | `FRONT_CAMERA_OFFSET` | `SIDE_CAMERA_OFFSET` |
| Telemetry prefix | `Vision/Front` | `Vision/Side` |

| Coprocessor | |
|---|---|
| Static IP | `10.7.51.11` (`PhotonVisionConstants.COPROCESSOR_IP`) |
| PhotonVision version | **exactly `v2026.3.4`**, the version in `vendordeps/photonlib.json` |
| Web UI (on robot network) | `http://10.7.51.11:5800` |
| Web UI (USB tether to RIO) | `http://roborio-751-frc.local:5800` or `http://172.22.11.2:5800`. On the real robot only (`isReal()`), `Robot.robotInit()` forwards port 5800 to the coprocessor. That's **the UI only**: camera preview streams use other ports, so calibrate over the radio or Ethernet. |

## Data Flow

```
PhotonVision coprocessor (10.7.51.11)
  finds tags, runs the multi-tag solve, publishes PhotonPipelineResults over NT
        │
        ▼
PhotonVisionSubsystem.periodic()          ← runs before Odometry (Odometry.getInstance() creates it first)
  per camera, inside try/catch:
    camera.getAllUnreadResults()          ← drains the queue, exactly once per loop
    for each frame:
      estimateCoprocMultiTagPose()        ← preferred: solved from every visible tag, far harder to "flip"
      else estimateLowestAmbiguityPose()  ← single-tag fallback
      → VisionObservation(estimate, tagIds, avgTagDistanceMeters, ambiguity, reprojErrPx)
    cache the list; publish Vision/<Cam>/* if frames arrived
    camera disconnected or drain threw → clear Vision/<Cam>/* telemetry
        │
        ▼
Odometry.periodic()
  if Pigeon yaw rate > MAX_ANGULAR_VELOCITY_DPS → skip fusing this loop (Odometry/VisionRejected)
  else for each cached observation (front, then side):
    getRejectReason()  → non-null? write Vision/<Cam>/RejectReason, RejectedCount++, skip
    stability: vision x/y vs drive.samplePoseAt(capture time), checked BEFORE fusing
    computeStdDevs()   → base (multi-tag, or single-tag for single tags / far tag pairs) × (1 + d²/30)
    drive.addVisionMeasurement(
        Pose2d(vision x, vision y, gyro heading),
        Utils.fpgaToCurrentTime(estimate.timestampSeconds),
        stdDevs)
    AcceptedCount++
  per camera: nothing accepted for POSE_STABLE_TIMEOUT_S → stability count = 0; publish counts
        │
        ▼
CTRE SwerveDrivetrain pose estimator → SwerveSubsystem.getPose() → PathPlanner, auto-aim, Field2d
```

Why it's built this way:

- **Created in `robotInit()`, before the first scheduler loop.** `Robot.robotInit()` calls `Odometry.getInstance()`, so vision fuses while **disabled and in auto**. `resetOdom: false` autos depend on vision having seeded the pose before the match starts. Subsystem `periodic()`s run in registration order, so `Odometry.getInstance()` creates `PhotonVisionSubsystem` first, then Odometry (its constructor is private). That way PV drains this loop's frames before Odometry reads them.
- **Drained once per loop, always.** `getAllUnreadResults()` empties a queue. It is called in `PhotonVisionSubsystem.periodic()` even while Odometry is skipping fusion (for example, mid-spin), so stale frames never pile up and photonlib's disconnect and time-sync alerts keep updating. `getFrontObservations()` / `getSideObservations()` are plain getters with no side effects.
- **Never crashes the loop.** photonlib throws if the coprocessor's PhotonVision message format doesn't match the vendordep (i.e. a different PV version). Each camera's drain is wrapped in `try/catch`. On an error, that camera reports no observations and a `PhotonVision camera '<name>' failed: ...` error goes to the DS at most every 5 s. The other camera and the rest of the scheduler keep running.
- **Tags, distance, and quality are computed by hand.** photonlib's `EstimatedRobotPose.targetsUsed` lists *every* target in the frame, not just the ones the solve used. For multi-tag, `VisionObservation.tagIds` comes from the multi-tag result's `fiducialIDsUsed`, the distance is averaged over just those tags (`+Infinity` if none of them are in the frame, which Odometry rejects as `non-finite`), and `reprojErrPx` is the solve's reprojection error. For single-tag, `tagIds` is the one target the fallback picked, using photonlib's exact rule (lowest ambiguity below 10, skipping `-1`), and the ambiguity and distance come from that target. `tagCount()` is `tagIds.size()`.
- **Strategy decides the trust, not tag count.** The single-tag fallback can run on a frame with several tags in view (for example, if multi-tag is switched off in the PV UI). Basing trust on strategy keeps a single-tag solve from getting multi-tag trust. One exception the other way: a multi-tag solve from only **2 tags averaging more than `FAR_TAG_PAIR_DIST_M` (5 m)** away gets single-tag trust, because two far tags on one face are almost one flat target and can flip like a single tag.
- **Sim only: frames from before a pose reset are dropped.** In sim, a pose reset teleports the robot, so a frame captured before it shows a spot the robot is no longer at. `SwerveSubsystem` stamps `getLastResetFpgaTime()` on every *sim* pose reset (`resetPose()` from PathPlanner's `resetOdom` or the Circle button, and the perspective rotation in `teleopInit()` on red), and those frames are rejected as `before reset`. On the real robot the stamp is never set, so this check never fires and resets behave exactly as they did before PhotonVision.
- **Stability is checked at capture time.** Each accepted estimate is compared with `drive.samplePoseAt(capture time)` (falls back to `getPose()` if the buffer has no sample), not the current pose: at full speed the robot moves ~0.2 m during camera latency. It's sampled **before** `addVisionMeasurement()`, so a frame's own correction can't make it agree with itself.
- **Timestamps.** PhotonVision time-syncs with the RIO (photonlib's TimeSyncServer, which starts when the first `PhotonCamera` is constructed), so `timestampSeconds` is FPGA time. CTRE's `addVisionMeasurement` uses the `Utils.getCurrentTimeSeconds()` timebase, so `Utils.fpgaToCurrentTime()` converts between the two.

## Camera Transforms (WPILib Convention)

`FRONT_CAMERA_OFFSET` and `SIDE_CAMERA_OFFSET` are `Transform3d`s from the **robot center at floor level** to the **camera lens**, in the **WPILib** robot frame:

| Axis | Positive direction |
|---|---|
| X | forward |
| Y | **LEFT** |
| Z | up |
| Pitch (rotation about Y) | **nose DOWN**. A camera tilted *up* 30° has pitch **−30°**. |
| Yaw (rotation about Z) | **LEFT** (counter-clockwise seen from above). Lens out the left side = **+90°**, out the right side = **−90°**, backward = 180°. |

`Rotation3d` takes `(roll, pitch, yaw)` in **radians**. Use `Units.degreesToRadians()`.

> **Warning: don't borrow Limelight numbers.** Limelight's camera-pose fields use their own conventions. The old `LimelightConstants` had its Y-sign comments flipped once (commit `884cd31`) without the values changing, so nobody knows which sign those numbers were in. The current values are copied from those old mounts and marked `TODO(measure)`. The pitch was converted to WPILib convention, but the Y values were copied as-is, so **Y may be mirrored on both cameras**. Measure the PhotonVision mounts fresh, in the convention above (tape measure or CAD).

> **Sim can't catch a wrong sign.** `PhotonVisionSim` places each fake camera with the same transform the estimator uses to solve, so a wrong transform still looks perfect in sim. Only the [on-robot validation](#on-robot-validation) catches it.

Current placeholder values:

| | X | Y | Z | Roll | Pitch | Yaw |
|---|---|---|---|---|---|---|
| `FRONT_CAMERA_OFFSET` | 0.0416 m | −0.1453 m (**sign unverified**) | 0.4128 m | 0 | −30° (tilted up) | 0 |
| `SIDE_CAMERA_OFFSET` | −0.2539 m | +0.1119 m (**sign unverified**) | 0.2906 m | 0 | 0 | −90° (**sign unverified**) |

The side camera's Y says it sits left of center, but its yaw says it faces right. That's possible, but check both when you measure.

## Filters and Trust

All constants are in `PhotonVisionConstants.java`. The filters run in `Odometry.getRejectReason()`, in this order. The first one that fails is written to `Vision/<Cam>/RejectReason` and counted in `Vision/<Cam>/RejectedCount`. `RejectReason` is **only written on a rejection**, so it shows the last reason even while later frames are fused.

| Constant | Value | What it does | `RejectReason` |
|---|---|---|---|
| — | | Reject if X, Y, Z, the timestamp, or the average tag distance is NaN or infinite. Every later check is "reject if value > limit", and a NaN fails every comparison, so it would otherwise get through and poison the pose. | `non-finite` |
| `MAX_LATENCY_S` | 0.5 s | Reject if capture-to-now is negative or longer than this. Usually means time sync with the coprocessor is broken. | `latency` |
| — | | Reject if the frame was captured before the last pose reset (`SwerveSubsystem.getLastResetFpgaTime()`). Normal for a moment after a reset. Checked after latency, so broken time sync shows as `latency`, not this. | `before reset` |
| `MAX_Z_ERROR_M` | 0.25 m | Reject if the estimated robot is floating above or sunk below the floor. Usually a bad solve or a wrong camera pitch. | `not on floor` |
| `FIELD_BORDER_MARGIN_M` | 0.5 m | Reject if X/Y is outside `[0, FIELD_LENGTH] × [0, FIELD_WIDTH]` by more than this. | `off field` |
| `MAX_AMBIGUITY` | 0.2 | **Single-tag only.** Reject if the used tag's pose ambiguity is above this, or isn't finite. PV's docs say anything over 0.2 is likely a flipped solve. | `ambiguous` |
| `MAX_SINGLE_TAG_DIST_M` | 4.0 m | **Single-tag only.** Past this, one tag covers too few pixels to trust. | `single tag too far` |
| `MAX_ANGULAR_VELOCITY_DPS` | 720 °/s | Skip fusing *everything* this loop while the Pigeon reports a faster spin (frames are still drained and dropped, and not counted as accepted or rejected). Rarely trips: teleop max is ~396 °/s, PathPlanner max 540 °/s. | n/a (`Odometry/VisionRejected` = true) |
| `SINGLE_TAG_STD_DEVS` | (4.0, 4.0, 99999) | Base x/y/θ std devs for `LOWEST_AMBIGUITY` estimates and far tag pairs. From PV's official example. | |
| `MULTI_TAG_STD_DEVS` | (0.5, 0.5, 99999) | Base std devs for `MULTI_TAG_PNP_ON_COPROCESSOR` estimates. From PV's official example. | |
| `FAR_TAG_PAIR_DIST_M` | 5.0 m | A multi-tag solve with **≤ 2 tags** and average distance **above** this gets `SINGLE_TAG_STD_DEVS` instead. | |
| `STD_DEV_DISTANCE_DIVISOR` | 30.0 | Std devs are multiplied by `1 + d² / 30`, where d is the average distance to the tags used. Bigger = distance matters less. | |
| `POSE_STABLE_EPSILON_METERS` | 0.10 m | An accepted estimate within this distance of where odometry thought the robot was **at capture time** counts as "agreeing". Anything farther resets the count to 0. | |
| `POSE_STABLE_THRESHOLD` | 10 | Consecutive agreeing estimates before a camera counts as stable. Dashboard only, nothing gates on it. | |
| `POSE_STABLE_TIMEOUT_S` | 0.5 s | A camera with no accepted estimate for this long has its count reset to 0 (stops claiming stable). | |

Higher std dev = trust vision **less**. θ is always `99999`, so heading comes from the Pigeon. Example x/y std devs after distance scaling:

| Avg tag distance | Scale | Multi-tag, 3+ tags | Multi-tag, 2 tags | Single-tag |
|---|---|---|---|---|
| 1 m | 1.03 | 0.52 m | 0.52 m | 4.1 m |
| 2 m | 1.13 | 0.57 m | 0.57 m | 4.5 m |
| 3 m | 1.30 | 0.65 m | 0.65 m | 5.2 m |
| 4 m | 1.53 | 0.77 m | 0.77 m | 6.1 m |
| 6 m | 2.20 | 1.10 m | 8.8 m (far pair) | rejected |

**Tuning**: If the pose is jittery while tags are in view, raise the base std devs. If it's slow to snap back after wheel slip, lower them, or check `Strategy`, since multi-tag may be silently off. Retune using `Sim/PoseErrorMeters` in sim and the on-robot tests below. Don't tune by feel during a match.

## Telemetry (AdvantageScope)

Everything goes to NetworkTables, and `DataLogManager` records all NT data to a `.wpilog` on the RIO, so you can open a match log in AdvantageScope afterwards.

Struct topics are published at the **NT root** (no `SmartDashboard/` prefix). Scalar topics are under **`SmartDashboard/`**.

**Per camera** (`<Cam>` = `Front` or `Side`):

| NT path | Type | Meaning |
|---|---|---|
| `Vision/<Cam>/RawPose` | `Pose2d[]` | Every estimate from this loop's frames, before filtering. Drop on the 2D/3D field next to the fused pose. |
| `Vision/<Cam>/RawPose3d` | `Pose3d[]` | Same, with Z / roll / pitch kept. **A wrong camera transform shows up here first** (Z ≠ 0, roll/pitch ≠ 0). |
| `Vision/<Cam>/TagsUsed` | `Pose3d[]` | Field poses of **only the tags the solve used** (`tagIds`), not every tag in view. Use AdvantageScope's 3D-field "Vision Target" object to draw lines to them. |
| `SmartDashboard/Vision/<Cam>/Connected` | boolean | `camera.isConnected()`, updated every loop |
| `SmartDashboard/Vision/<Cam>/Strategy` | string | `MULTI_TAG_PNP_ON_COPROCESSOR`, `LOWEST_AMBIGUITY`, or `NONE`. `LOWEST_AMBIGUITY` with 2+ tags in view means multi-tag is off in that PV pipeline. |
| `SmartDashboard/Vision/<Cam>/TagCount` | number | Tags used by the newest estimate |
| `SmartDashboard/Vision/<Cam>/AvgTagDist` | number (m) | Average distance to those tags |
| `SmartDashboard/Vision/<Cam>/Ambiguity` | number | Single-tag pose ambiguity of the newest estimate. **Always 0 for multi-tag** (one solution, nothing to flip to). |
| `SmartDashboard/Vision/<Cam>/ReprojErrPx` | number (px) | Multi-tag reprojection error of the newest estimate. NaN for single-tag. High = bad calibration. |
| `SmartDashboard/Vision/<Cam>/LatencyMs` | number (ms) | Capture-to-now of the newest estimate |
| `SmartDashboard/Vision/<Cam>/RejectReason` | string | The **last** reason an estimate was rejected. Only written on rejection, so it stays put while good frames are fused. Compare with the counts below to tell if it's current. |
| `SmartDashboard/Vision/<Cam>/AcceptedCount` | number | Estimates fused since boot (published by Odometry every loop) |
| `SmartDashboard/Vision/<Cam>/RejectedCount` | number | Estimates rejected since boot. Climbing while `AcceptedCount` is flat = check `RejectReason`. |

The raw topics (`RawPose`, `RawPose3d`, `TagsUsed`, `Strategy`, `TagCount`, `AvgTagDist`, `Ambiguity`, `ReprojErrPx`, `LatencyMs`) only update when new frames arrive. A loop with no new frames leaves the old values in place, so the poses don't flicker. New frames with no estimate clear the struct arrays, set `Strategy` = `NONE` and `TagCount` = 0, and set the other numbers to NaN. The same clear happens **every loop the camera is disconnected or its drain threw**, so AdvantageScope never keeps drawing a dead camera's last pose.

**Odometry and drive**:

| NT path | Meaning |
|---|---|
| `SmartDashboard/Field` | Field2d with the fused pose (what auto and auto-aim use) |
| `SmartDashboard/Swerve/Pose x`, `Pose y`, `Rotation` | Fused pose from `SwerveSubsystem.getPose()` |
| `SmartDashboard/Odometry/VisionRejected` | true while the spin gate is skipping fusion |
| `SmartDashboard/Odometry/FrontStable`, `SideStable`, `PoseStable` | Stability counters (`PoseStable` = both) |
| `SmartDashboard/Sim/PoseErrorMeters` | Sim only: distance between the fused pose and MapleSim ground truth |
| `photonvision/<nickname>/...` | photonlib's raw pipeline results, published by the coprocessor |

photonlib also raises WPILib Alerts in the `PhotonAlerts` group (camera disconnected, time sync). Elastic and AdvantageScope both show them.

**Quick diagnosis:**

| Symptom | Look at |
|---|---|
| Fused pose never moves toward vision | `Connected`, then `Strategy` (`NONE` = no tags), then `AcceptedCount` / `RejectedCount` and `RejectReason`, then DS errors for a version mismatch |
| Raw poses blank and numbers NaN with tags in view | `Connected` is false, or a `PhotonVision camera ... failed` DS error (telemetry is cleared in both cases) |
| `RejectReason` = `latency` constantly | Time sync broken. Check the `PhotonAlerts` TimeSync alert and the PV version. |
| `RejectReason` = `before reset` | Sim only. Normal right after a pose reset (`resetOdom` auto start, Circle button, `teleopInit()` perspective change). Frames captured after the reset are fused normally. Can't happen on the real robot. |
| `RejectReason` = `non-finite` | A NaN/infinite solve, or multi-tag tag IDs missing from the frame. Shouldn't happen. If it repeats, check the PV version. |
| `RejectReason` = `not on floor` | Camera pitch or Z is wrong. Compare `RawPose3d` Z/pitch against reality. |
| `ReprojErrPx` high on multi-tag | Calibration is off (recalibrate at the competition resolution), or the PV field layout doesn't match `FIELD_LAYOUT` / the real field |
| Raw poses from one camera are offset by a constant amount | That camera's translation is wrong |
| Raw poses from the two cameras are 180° apart in heading | Side camera yaw sign is wrong (+90 vs −90) |
| Pose jumps when only one tag is visible | Single-tag flip. Should be caught by `ambiguous`. If not, consider lowering `MAX_AMBIGUITY`. |
| `FrontStable` / `SideStable` never true with tags in view | That camera disagrees with odometry by more than 0.10 m. A constant offset usually means a wrong camera transform. |

## Coprocessor Bring-up Checklist

Do this once per coprocessor, and again after reimaging or swapping cameras.

- [ ] **PhotonVision version is exactly `v2026.3.4`**, the same as `vendordeps/photonlib.json`. A different version can change the NT message format, which makes photonlib throw. The code catches it, but that camera then gets **no** vision.
- [ ] **Networking**: team number `751`, **static IP `10.7.51.11`** (matches `COPROCESSOR_IP`), and a unique hostname (not the default `photonvision`).
- [ ] **Camera nicknames are exactly `front` and `side`** (lowercase). A typo doesn't error: the `PhotonCamera` just subscribes to an empty NT path and `Connected` stays false.
- [ ] **Label the USB ports and cameras.** PhotonVision matches each camera to its settings and calibration by USB port. Swapping cables can swap `front`/`side` or apply the wrong calibration ("Camera Mismatch" status in the UI).
- [ ] **Calibrate each camera at the resolution used in competition**, with a ChArUco board (Cameras tab), over the radio or Ethernet (the USB-tether port forward only carries the UI, not the camera streams). Measure the printed board with calipers and enter the real square and marker sizes. Calibration is per physical camera *and* per resolution. A pipeline running an uncalibrated resolution can't do 3D.
- [ ] **Every pipeline**: AprilTag, **3D mode** on, and **"Do Multi-Target Estimation" ON** (Output tab). Without it, every estimate falls back to single-tag (`Strategy` = `LOWEST_AMBIGUITY`).
- [ ] **PV UI field layout matches `PhotonVisionConstants.FIELD_LAYOUT`.** The multi-tag solve uses the PV layout (Settings tab → "AprilTag Field Layout" card), and the single-tag fallback uses `FIELD_LAYOUT` on the RIO. Both default to the 2026 **welded** layout (`k2026RebuiltWelded`). If an event uses the AndyMark field, change **both**: import the AndyMark JSON in PV (Settings → Device Control → Import Settings → AprilTag Layout) and switch `FIELD_LAYOUT` to `k2026RebuiltAndymark`.
- [ ] **No TimeSync alert** in `PhotonAlerts` and no PV version-mismatch errors on the DS.
- [ ] **Power**: run the coprocessor off its own dedicated 5 V regulator. If it browns out, it reboots mid-match and vision is gone until it finishes booting.
- [ ] **Camera transforms measured** in WPILib convention and entered in `PhotonVisionConstants` (remove the `TODO(measure)`s). Check the Y sign on **both** cameras.

## On-Robot Validation

Do this after the checklist and after any camera mount change. Watch `RawPose3d` on AdvantageScope's 3D field, live over NT.

1. **Preflight.** `Vision/Front/Connected` and `Vision/Side/Connected` are true. There are no `PhotonAlerts` and no `PhotonVision camera ... failed` errors on the DS. With 2+ tags in view, `Strategy` reads `MULTI_TAG_PNP_ON_COPROCESSOR` and `ReprojErrPx` stays low.
2. **One camera at a time** (cover the other lens rather than unplugging it, because ports matter). Park the robot at a **tape-measured** spot on the field.
   - `RawPose3d` X/Y within ~5 cm of the tape measurement
   - Z ≈ 0, roll ≈ 0, pitch ≈ 0. A non-zero pitch or Z means the camera pitch, or its sign, is wrong.
3. **Spin in place** slowly while the camera keeps a tag in view. With a correct transform, the raw estimate **stays still**. If it **traces a circle**, the translation (or a sign in it, e.g. a mirrored Y) is wrong. The circle's radius is roughly the size of the error.
4. **Both cameras together.** Park where both see tags. Front and side raw X/Y should agree within a few cm. If their headings differ by exactly **180°**, the side camera's yaw sign is wrong (+90 vs −90). **Compare the cameras to each other, not to the gyro**: on red, before teleop, the gyro heading can legitimately be 180° off until `teleopInit()` sets the operator perspective.
5. **Drive around.** `RejectedCount` should climb on far or ambiguous single tags (`RejectReason` = `single tag too far` / `ambiguous`), `AcceptedCount` should climb whenever good tags are in view, and the fused pose (`Field`) shouldn't jump.
6. **Auto from disabled.** Boot the robot and leave it **disabled** at a starting spot with tags in view. Check that the fused pose snaps to the right place before enabling, then run a `resetOdom: false` auto (e.g. `Blue-Left`, `Red-Right`, `blue-left-depot`). The HIPPOS autos keep `resetOdom` on on purpose. This is the only real test of a `resetOdom: false` auto: sim can check the seeding but not the path (see below).

## Simulation

`PhotonVisionSim` (sim only, created in `robotInit()` when `Utils.isSimulation()`) runs a photonlib `VisionSystemSim` using `PhotonVisionConstants.FIELD_LAYOUT` (passed to each `PhotonCameraSim` too, so the simulated multi-tag solve doesn't fall back to WPILib's default field), with both camera transforms, from MapleSim's **ground-truth** pose (`SwerveSubsystem.getGroundTruthPose()`). The fake cameras publish NT data shaped exactly like the coprocessor's, so `PhotonVisionSubsystem` and `Odometry` run unchanged.

`SwerveSubsystem.SIM_USE_ESTIMATED_POSE` (default `true`) makes `getPose()` in sim return the CTRE estimator, like the real robot, so sim actually exercises vision fusion. Watch `Sim/PoseErrorMeters`. It should stay small while driving and shrink back after you ram a wall. Set it to `false` to test mechanisms against a perfect pose.

Pose resets in sim:

- `resetPose()` teleports the MapleSim robot (`setSimulationWorldPose`), calls `PhotonVisionSim.onTeleport()` to reset the fake cameras' pose history (otherwise they'd render frames from partway along the jump), waits 50 ms, then resets the CTRE estimator.
- The operator-perspective change in `teleopInit()` rotates the ground truth **and** the estimate in place, separately. It doesn't teleport the truth onto the estimate, so any existing pose error is kept.
- **The sim robot always spawns at (2.5, 4, 0°).** A sim run of a `resetOdom: false` auto from disabled only verifies that vision **seeds** the pose to (2.5, 4). The auto then drives from there, not from its real start spot, so it says nothing about the path itself.
- **Select an alliance in the Sim GUI before enabling teleop.** `teleopInit()` calls `DriverStation.getAlliance().get()` without checking it's present.

Both sim cameras use the `PI4_LIFECAM_640_480` profile as a placeholder until the real camera model and calibration exist. See [Simulation](simulation.md).

## Code Reference

| File | Key API |
|---|---|
| `subsystems/vision/PhotonVisionConstants.java` | Camera names, `COPROCESSOR_IP`, `FIELD_LAYOUT`, camera transforms, filter/std-dev/stability constants |
| `subsystems/vision/PhotonVisionSubsystem.java` | `getInstance()`, `getFrontObservations()`, `getSideObservations()`, `record VisionObservation(estimate, tagIds, avgTagDistanceMeters, ambiguity, reprojErrPx)` with `tagCount()` |
| `subsystems/drive/Odometry.java` | `getInstance()` (creates `PhotonVisionSubsystem` first; constructor is private), `getPose()`, `getYaw()`, `resetPose(Pose2d)`, `isPoseStable()` / `isFrontStable()` / `isSideStable()` |
| `subsystems/drive/SwerveSubsystem.java` | `resetPose(Pose2d)` (sim: also teleports MapleSim + vision sim), `getLastResetFpgaTime()`, `samplePoseAt()` (CTRE), `getGroundTruthPose()` |
| `subsystems/vision/PhotonVisionSim.java` | Sim-only `VisionSystemSim` driven by ground truth; static `onTeleport(Pose2d)` |

## Known Gaps

- Camera transforms are placeholders. The Y sign on both cameras and the side yaw sign are unverified.
- Heading is never taken from vision. Seeding or correcting the gyro from multi-tag (e.g. `PNP_DISTANCE_TRIG_SOLVE` + `addHeadingData`) isn't implemented yet.
- The Circle button (`SwerveSubsystem.setRobotRotationByAlliance()`) resets the pose to `(0, 0, rot)`. Vision then has to drag it back across the field. The fix is to keep the current translation. That's a separate change.
- The sim camera model is a placeholder (see above).
- The sim robot can't start at an auto's start pose (see above).

## External Docs

- [PhotonVision docs](https://docs.photonvision.org/en/latest/)
- [Coprocessor networking](https://docs.photonvision.org/en/latest/docs/quick-start/networking.html)
- [Camera calibration](https://docs.photonvision.org/en/latest/docs/calibration/calibration.html)
- [Multi-tag localization](https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/multitag.html)
- [Camera matching (USB ports)](https://docs.photonvision.org/en/latest/docs/quick-start/camera-matching.html)
- [Robot pose estimation (photonlib)](https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html)
- [WPILib coordinate system](https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html)
