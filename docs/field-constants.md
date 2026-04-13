# Field Constants

This document describes the 2026 REBUILT field geometry and how it's represented in code.

## Overview

`FieldConstants.java` defines all field element positions, dimensions, and reference points for the 2026 REBUILT game. All coordinates use the **WPILib field coordinate system** (Blue alliance origin at bottom-right of the Blue wall, +X toward Red wall, +Y toward left wall).

## Coordinate System

```
                    RED ALLIANCE WALL
    ┌─────────────────────────────────────────┐
    │                                         │  ↑
    │              fieldLength                │  │ +Y (left from Blue wall)
    │         ◄────────────────►              │  │
    │                                         │  │
    │   [Red Hub]                [Blue Hub]   │  │ fieldWidth
    │                                         │  │
    │   [Red Tower]              [Blue Tower] │  │
    │                                         │  │
    ├─────────────────────────────────────────┤  │
    │            BLUE ALLIANCE WALL           │  │
    └─────────────────(0,0)──────────────────┘  +X (toward Red) →
```

The origin (0,0) is at the bottom-right corner of the BLUE alliance wall. "Alliance side" constants refer to BLUE; "opp" constants refer to RED.

## Field Dimensions

Loaded dynamically from the AprilTag layout JSON:
- `fieldLength` — length of the field (Blue wall to Red wall), from `AprilTagFieldLayout.getFieldLength()`
- `fieldWidth` — width of the field, from `AprilTagFieldLayout.getFieldWidth()`

## Field Elements

### Hub

Each alliance has one **hub** — a 47-inch cube scoring target where fuel is deposited.

| Property | Value |
|----------|-------|
| Width | 47 inches (1.194 m) |
| Height | 72 inches (1.829 m) including catcher |
| Inner width | 41.7 inches (1.059 m) |
| Inner height | 56.5 inches (1.435 m) |

**Blue Hub** position derived from AprilTag 26 (near face):
- `Hub.topCenterPoint` — 3D center at full height
- `Hub.innerCenterPoint` — 3D center at inner scoring height
- Corner positions: `nearLeftCorner`, `nearRightCorner`, `farLeftCorner`, `farRightCorner`
- Face poses (from AprilTags): near=26, far=20, right=18, left=21

**Red Hub** position derived from AprilTag 4:
- `Hub.oppTopCenterPoint` — 3D center at full height
- Corner positions: `oppNearLeftCorner`, etc.

### Bumps (Ramps)

Two ramps adjacent to each hub, one on each side:

**Left Bump** (higher Y values):
| Property | Value |
|----------|-------|
| Width | 73 inches (1.854 m) |
| Height | 6.513 inches (0.165 m) |
| Depth | 44.4 inches (1.128 m) |

**Right Bump** (lower Y values): Same dimensions, mirrored position.

### Trenches

Openings in the field walls near each hub for fuel recovery:

**Left Trench** (higher Y, near left wall):
| Property | Value |
|----------|-------|
| Width | 65.65 inches (1.667 m) |
| Depth | 47 inches (1.194 m) |
| Height | 40.25 inches (1.022 m) |
| Opening width | 50.34 inches (1.279 m) |
| Opening height | 22.25 inches (0.565 m) |

**Right Trench** (lower Y, near right wall): Same dimensions.

### Tower (Climbing Structure)

Each alliance has a tower with three climbing rungs:

| Property | Value |
|----------|-------|
| Width | 49.25 inches (1.251 m) |
| Depth | 45 inches (1.143 m) |
| Height | 78.25 inches (1.988 m) |
| Inner opening | 32.25 inches (0.819 m) |
| Upright height | 72.1 inches (1.831 m) |
| Front face X | 43.51 inches (1.105 m) from alliance wall |

**Rung Heights:**
| Level | Height | Teleop Points | Auto Points |
|-------|--------|---------------|-------------|
| Low (L1) | 27 inches (0.686 m) | 10 | 15 |
| Mid (L2) | 45 inches (1.143 m) | 20 | — |
| High (L3) | 63 inches (1.600 m) | 30 | — |

Blue tower derived from AprilTag 31, Red tower from AprilTag 15.

### Depot

Raised fuel storage area on the alliance wall:

| Property | Value |
|----------|-------|
| Width | 42 inches (1.067 m) |
| Depth | 27 inches (0.686 m) |
| Height | 1.125 inches (0.029 m) |
| Distance from center Y | 75.93 inches (1.929 m) |

### Outpost

Human player station on the alliance wall:

| Property | Value |
|----------|-------|
| Width | 31.8 inches (0.808 m) |
| Opening height from floor | 28.1 inches (0.714 m) |
| Height | 7 inches (0.178 m) |

Blue outpost derived from AprilTag 29.

## Vertical Reference Lines

Key X-axis boundaries (defined in `LinesVertical`):

| Line | Derived From | Description |
|------|-------------|-------------|
| `center` | `fieldLength / 2` | Field center line |
| `starting` / `allianceZone` | AprilTag 26 X | Blue alliance zone boundary |
| `hubCenter` | Tag 26 X + hub width/2 | Blue hub center X |
| `neutralZoneNear` | center - 120" | Near edge of neutral zone |
| `neutralZoneFar` | center + 120" | Far edge of neutral zone |
| `oppHubCenter` | Tag 4 X + hub width/2 | Red hub center X |
| `oppAllianceZone` | AprilTag 10 X | Red alliance zone boundary |

## Horizontal Reference Lines

Key Y-axis boundaries (defined in `LinesHorizontal`):

| Line | Description |
|------|-------------|
| `center` | `fieldWidth / 2` |
| `rightBumpStart` | Hub near-right corner Y |
| `rightBumpEnd` | Right bump far edge |
| `leftBumpEnd` | Hub near-left corner Y |
| `leftBumpStart` | Left bump far edge |
| Trench open regions between bumps and field walls |

## AprilTag Layout

### Layout Types

```java
public enum AprilTagLayoutType {
    OFFICIAL("2026-official"),  // Full field layout
    NONE("2026-none");         // Empty layout (for testing)
}
```

### Layout Loading

The AprilTag layout JSON is loaded lazily with double-checked locking:

```java
Path p = Constants.disableHAL
    ? Path.of("src", "main", "deploy", "apriltags", fieldType.getJsonFolder(), name + ".json")
    : Path.of(Filesystem.getDeployDirectory().getPath(), "apriltags", fieldType.getJsonFolder(), name + ".json");
layout = new AprilTagFieldLayout(p);
```

When running on the robot (`HAL` is available), it reads from the deploy directory (`/home/lvuser/deploy/`). In unit tests (HAL disabled), it reads from the source tree.

### Field Type

```java
public enum FieldType {
    ANDYMARK("andymark"),   // AndyMark field variant
    WELDED("welded");       // Welded field variant (currently used)
}

public static final FieldType fieldType = FieldType.WELDED;
```

### Key AprilTag IDs

| ID | Location |
|----|----------|
| 26 | Blue hub near face |
| 20 | Blue hub far face |
| 18 | Blue hub right face |
| 21 | Blue hub left face |
| 4  | Red hub near face |
| 10 | Red alliance zone |
| 31 | Blue tower |
| 15 | Red tower |
| 29 | Blue outpost |

## Using Field Constants

### In Autonomous

```java
// Drive to the Blue hub scoring position
Pose2d scoringPosition = new Pose2d(
    FieldConstants.Hub.nearFace.getX() - 1.0,  // 1 meter in front of hub
    FieldConstants.Hub.nearFace.getY(),
    FieldConstants.Hub.nearFace.getRotation()
);
```

### In Simulation

The `Arena2026Rebuilt` simulation class uses similar field constants to create physics obstacles for the simulated field.

## Dependencies

`FieldConstants` uses Lombok annotations:
- `@Getter` on enum fields
- `@RequiredArgsConstructor` on enums

These are compile-time annotations via the Lombok dependency in `build.gradle`.
