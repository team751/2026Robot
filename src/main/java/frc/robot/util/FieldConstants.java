// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.util;

import com.fasterxml.jackson.databind.ObjectMapper;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.IOException;
import java.nio.file.Path;
import lombok.Getter;
import lombok.RequiredArgsConstructor;

/**
 * Contains positions and dimensions for all field elements and reference points.
 *
 * <p>NOTE: All constants are defined in the WPILib field coordinate system. The origin (0,0) is at
 * the bottom-right corner of the BLUE alliance wall. All "alliance side" constants refer to the
 * BLUE alliance, and all "opp" (opposing) constants refer to the RED alliance.
 */
public class FieldConstants {
  public static final FieldType fieldType = FieldType.WELDED;

  // --- AprilTag Configuration ---
  public static final int aprilTagCount = AprilTagLayoutType.OFFICIAL.getLayout().getTags().size();
  public static final double aprilTagWidth = Units.inchesToMeters(6.5);
  public static final AprilTagLayoutType defaultAprilTagType = AprilTagLayoutType.OFFICIAL;

  // --- Field Dimensions ---
  public static final double fieldLength = AprilTagLayoutType.OFFICIAL.getLayout().getFieldLength();
  public static final double fieldWidth = AprilTagLayoutType.OFFICIAL.getLayout().getFieldWidth();

  /**
   * Key vertical lines on the field defined by their X-axis position. Used for zone boundaries and
   * alignment references. "Near" = closer to the BLUE wall, "Far" = closer to the RED wall.
   */
  public static class LinesVertical {
    public static final double center = fieldLength / 2.0;

    // BLUE alliance zone boundary (X of tag 26, on the BLUE hub's near face)
    public static final double starting =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX();
    public static final double allianceZone = starting;

    // BLUE hub center X (tag 26 X + half the hub width)
    public static final double hubCenter =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + Hub.width / 2.0;

    // Neutral zone boundaries (120 inches on each side of center)
    public static final double neutralZoneNear = center - Units.inchesToMeters(120);
    public static final double neutralZoneFar = center + Units.inchesToMeters(120);

    // RED hub center X (tag 4 X + half the hub width)
    public static final double oppHubCenter =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + Hub.width / 2.0;

    // RED alliance zone boundary (X of tag 10)
    public static final double oppAllianceZone =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(10).get().getX();
  }

  /**
   * Key horizontal lines on the field defined by their Y-axis position. "Left" and "right" are from
   * the perspective of a driver standing at the BLUE alliance wall looking across the field.
   */
  public static class LinesHorizontal {

    public static final double center = fieldWidth / 2.0;

    // Right side of hub (lower Y values)
    public static final double rightBumpStart = Hub.nearRightCorner.getY();
    public static final double rightBumpEnd = rightBumpStart - RightBump.width;
    public static final double rightTrenchOpenStart = rightBumpEnd - Units.inchesToMeters(12.0);
    public static final double rightTrenchOpenEnd = 0;

    // Left side of hub (higher Y values)
    public static final double leftBumpEnd = Hub.nearLeftCorner.getY();
    public static final double leftBumpStart = leftBumpEnd + LeftBump.width;
    public static final double leftTrenchOpenEnd = leftBumpStart + Units.inchesToMeters(12.0);
    public static final double leftTrenchOpenStart = fieldWidth;
  }

  /**
   * Hub dimensions and positions. Each alliance has one hub (a 47" cube scoring target). "Near" =
   * the face closest to that alliance's wall, "Far" = the face closest to the opposing wall.
   */
  public static class Hub {

    // --- Hub Dimensions (same for both BLUE and RED) ---
    public static final double width = Units.inchesToMeters(47.0);
    public static final double height =
        Units.inchesToMeters(72.0); // includes the catcher at the top
    public static final double innerWidth = Units.inchesToMeters(41.7);
    public static final double innerHeight = Units.inchesToMeters(56.5);

    // =========================================================================
    // BLUE HUB - Located on the blue alliance side of the field (low X values).
    //            Position is derived from AprilTag 26 on the hub's near face.
    // =========================================================================

    /** Top-center of the BLUE hub (3D, at full hub height). */
    public static final Translation3d topCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            height);

    /** Inner scoring target center of the BLUE hub. */
    public static final Translation3d innerCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            innerHeight);

    // BLUE hub footprint corners (2D, projected onto the floor)
    public static final Translation2d nearLeftCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d nearRightCorner =
        new Translation2d(topCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d farLeftCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d farRightCorner =
        new Translation2d(topCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);

    // BLUE hub face poses (from AprilTags on each face of the BLUE hub)
    /** BLUE hub near face (facing the blue alliance wall) - AprilTag 26. */
    public static final Pose2d nearFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(26).get().toPose2d();
    /** BLUE hub far face (facing field center) - AprilTag 20. */
    public static final Pose2d farFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(20).get().toPose2d();
    /** BLUE hub right face (lower Y) - AprilTag 18. */
    public static final Pose2d rightFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(18).get().toPose2d();
    /** BLUE hub left face (higher Y) - AprilTag 21. */
    public static final Pose2d leftFace =
        AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(21).get().toPose2d();

    // =========================================================================
    // RED HUB - Located on the red alliance side of the field (high X values).
    //           Position is derived from AprilTag 4 on the hub's near face.
    // =========================================================================

    /** Top-center of the RED hub (3D, at full hub height). */
    public static final Translation3d oppTopCenterPoint =
        new Translation3d(
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(4).get().getX() + width / 2.0,
            fieldWidth / 2.0,
            height);

    // RED hub footprint corners (2D, projected onto the floor)
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppNearRightCorner =
        new Translation2d(oppTopCenterPoint.getX() - width / 2.0, fieldWidth / 2.0 - width / 2.0);
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 + width / 2.0);
    public static final Translation2d oppFarRightCorner =
        new Translation2d(oppTopCenterPoint.getX() + width / 2.0, fieldWidth / 2.0 - width / 2.0);
  }

  /** Left bump (ramp) adjacent to the left side of each hub (higher Y values). */
  public static class LeftBump {

    // --- Dimensions ---
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // BLUE hub left bump corners
    public static final Translation2d nearLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
    public static final Translation2d farLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner = Hub.farLeftCorner;

    // RED hub left bump corners
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
  }

  /** Right bump (ramp) adjacent to the right side of each hub (lower Y values). */
  public static class RightBump {

    // --- Dimensions ---
    public static final double width = Units.inchesToMeters(73.0);
    public static final double height = Units.inchesToMeters(6.513);
    public static final double depth = Units.inchesToMeters(44.4);

    // BLUE hub right bump corners
    public static final Translation2d nearLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d nearRightCorner = Hub.nearLeftCorner;
    public static final Translation2d farLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d farRightCorner = Hub.farLeftCorner;

    // RED hub right bump corners
    public static final Translation2d oppNearLeftCorner =
        new Translation2d(LinesVertical.hubCenter + width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppNearRightCorner = Hub.oppNearLeftCorner;
    public static final Translation2d oppFarLeftCorner =
        new Translation2d(LinesVertical.hubCenter - width / 2, Units.inchesToMeters(255));
    public static final Translation2d oppFarRightCorner = Hub.oppFarLeftCorner;
  }

  /** Left trench (higher Y, toward the field's left wall when viewed from the BLUE wall). */
  public static class LeftTrench {

    // --- Dimensions ---
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    // BLUE side left trench opening
    public static final Translation3d openingTopLeft =
        new Translation3d(LinesVertical.hubCenter, fieldWidth, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(LinesVertical.hubCenter, fieldWidth - openingWidth, openingHeight);

    // RED side left trench opening
    public static final Translation3d oppOpeningTopLeft =
        new Translation3d(LinesVertical.oppHubCenter, fieldWidth, openingHeight);
    public static final Translation3d oppOpeningTopRight =
        new Translation3d(LinesVertical.oppHubCenter, fieldWidth - openingWidth, openingHeight);
  }

  /** Right trench (lower Y, toward the field's right wall when viewed from the BLUE wall). */
  public static class RightTrench {

    // --- Dimensions ---
    public static final double width = Units.inchesToMeters(65.65);
    public static final double depth = Units.inchesToMeters(47.0);
    public static final double height = Units.inchesToMeters(40.25);
    public static final double openingWidth = Units.inchesToMeters(50.34);
    public static final double openingHeight = Units.inchesToMeters(22.25);

    // BLUE side right trench opening
    public static final Translation3d openingTopLeft =
        new Translation3d(LinesVertical.hubCenter, openingWidth, openingHeight);
    public static final Translation3d openingTopRight =
        new Translation3d(LinesVertical.hubCenter, 0, openingHeight);

    // RED side right trench opening
    public static final Translation3d oppOpeningTopLeft =
        new Translation3d(LinesVertical.oppHubCenter, openingWidth, openingHeight);
    public static final Translation3d oppOpeningTopRight =
        new Translation3d(LinesVertical.oppHubCenter, 0, openingHeight);
  }

  /** Tower (climbing structure) with three rungs at different heights. One per alliance. */
  public static class Tower {

    // --- Dimensions ---
    public static final double width = Units.inchesToMeters(49.25);
    public static final double depth = Units.inchesToMeters(45.0);
    public static final double height = Units.inchesToMeters(78.25);
    public static final double innerOpeningWidth = Units.inchesToMeters(32.250);
    public static final double frontFaceX = Units.inchesToMeters(43.51);
    public static final double uprightHeight = Units.inchesToMeters(72.1);

    // --- Rung heights from the floor ---
    public static final double lowRungHeight = Units.inchesToMeters(27.0);
    public static final double midRungHeight = Units.inchesToMeters(45.0);
    public static final double highRungHeight = Units.inchesToMeters(63.0);

    // BLUE tower (near the blue alliance wall, derived from AprilTag 31)
    public static final Translation2d centerPoint =
        new Translation2d(
            frontFaceX, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY());
    public static final Translation2d leftUpright =
        new Translation2d(
            frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                + innerOpeningWidth / 2
                + Units.inchesToMeters(0.75));
    public static final Translation2d rightUpright =
        new Translation2d(
            frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(31).get().getY())
                - innerOpeningWidth / 2
                - Units.inchesToMeters(0.75));

    // RED tower (near the red alliance wall, derived from AprilTag 15)
    public static final Translation2d oppCenterPoint =
        new Translation2d(
            fieldLength - frontFaceX,
            AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY());
    public static final Translation2d oppLeftUpright =
        new Translation2d(
            fieldLength - frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                + innerOpeningWidth / 2
                + Units.inchesToMeters(0.75));
    public static final Translation2d oppRightUpright =
        new Translation2d(
            fieldLength - frontFaceX,
            (AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(15).get().getY())
                - innerOpeningWidth / 2
                - Units.inchesToMeters(0.75));
  }

  /** Depot - the raised fuel storage area on the alliance wall (BLUE side only defined here). */
  public static class Depot {

    // --- Dimensions ---
    public static final double width = Units.inchesToMeters(42.0);
    public static final double depth = Units.inchesToMeters(27.0);
    public static final double height = Units.inchesToMeters(1.125);
    public static final double distanceFromCenterY = Units.inchesToMeters(75.93);

    // BLUE depot reference points (on the blue alliance wall)
    public static final Translation3d depotCenter =
        new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY, height);
    public static final Translation3d leftCorner =
        new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY + (width / 2), height);
    public static final Translation3d rightCorner =
        new Translation3d(depth, (fieldWidth / 2) + distanceFromCenterY - (width / 2), height);
  }

  /** Outpost - the human player station on the alliance wall (BLUE side only defined here). */
  public static class Outpost {

    // --- Dimensions ---
    public static final double width = Units.inchesToMeters(31.8);
    public static final double openingDistanceFromFloor = Units.inchesToMeters(28.1);
    public static final double height = Units.inchesToMeters(7.0);

    // BLUE outpost center (on the blue alliance wall, derived from AprilTag 29)
    public static final Translation2d centerPoint =
        new Translation2d(0, AprilTagLayoutType.OFFICIAL.getLayout().getTagPose(29).get().getY());
  }

  /** Which physical field variant is being used (affects AprilTag layout JSON path). */
  @RequiredArgsConstructor
  public enum FieldType {
    ANDYMARK("andymark"),
    WELDED("welded");

    @Getter private final String jsonFolder;
  }

  /** AprilTag layout variants. Lazily loads the JSON layout from the deploy directory. */
  public enum AprilTagLayoutType {
    OFFICIAL("2026-official"),
    NONE("2026-none");

    private final String name;
    private volatile AprilTagFieldLayout layout;
    private volatile String layoutString;

    AprilTagLayoutType(String name) {
      this.name = name;
    }

    public AprilTagFieldLayout getLayout() {
      if (layout == null) {
        synchronized (this) {
          if (layout == null) {
            try {
              Path p =
                  Constants.disableHAL
                      ? Path.of(
                          "src",
                          "main",
                          "deploy",
                          "apriltags",
                          fieldType.getJsonFolder(),
                          name + ".json")
                      : Path.of(
                          Filesystem.getDeployDirectory().getPath(),
                          "apriltags",
                          fieldType.getJsonFolder(),
                          name + ".json");
              layout = new AprilTagFieldLayout(p);
              layoutString = new ObjectMapper().writeValueAsString(layout);
            } catch (IOException e) {
              throw new RuntimeException(e);
            }
          }
        }
      }
      return layout;
    }

    public String getLayoutString() {
      if (layoutString == null) {
        getLayout();
      }
      return layoutString;
    }
  }
}