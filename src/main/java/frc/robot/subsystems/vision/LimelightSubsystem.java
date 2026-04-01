// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LimelightHelpers;
import frc.robot.subsystems.drive.Odometry;
import limelight.Limelight;

/* Rough Overview of Vision/Limelight
 * The Limelight/Vision subsystem is used to help determine
 * where our robot is on the field. This is used in the
 * Odometry.java file.
 *
 * Mainly, this file is a collection of methods to
 * help Odometry.java do its thing.
 */

public class LimelightSubsystem extends SubsystemBase {
  private static LimelightSubsystem instance;
  private Limelight limelightFront;
  private Limelight limelightSide;

  public static LimelightSubsystem getInstance() {
    if (instance == null) instance = new LimelightSubsystem();
    return instance;
  }

  public void initLimsplz(){
    Limelight tmpFront;
    Limelight tmpSide;

    try {
      tmpFront = new Limelight(LimelightConstants.LimelightFront.name);
      tmpFront
        .getSettings()
          .withCameraOffset(
              new Pose3d(
                  LimelightConstants.LimelightFront.xOffset,
                  LimelightConstants.LimelightFront.yOffset,
                  LimelightConstants.LimelightFront.zOffset,
                  LimelightConstants.LimelightFront.rotationOffset));

    } catch (Throwable t) {
      System.err.println("Limelight Front init failed: " + t.toString());
      t.printStackTrace();
      tmpFront = null;
    }

    try {
      tmpSide = new Limelight(LimelightConstants.LimelightSide.name);
      tmpSide
        .getSettings()
          .withCameraOffset(
              new Pose3d(
                  LimelightConstants.LimelightSide.xOffset,
                  LimelightConstants.LimelightSide.yOffset,
                  LimelightConstants.LimelightSide.zOffset,
                  LimelightConstants.LimelightSide.rotationOffset));

    } catch (Throwable t) {
      System.err.println("Limelight Side init failed: " + t.toString());
      t.printStackTrace();
      tmpSide = null;
    }

    // Inits both limelights using their pre-set names
    limelightFront = tmpFront;

    limelightSide = tmpSide;

    // Sets the settings for each limelight with their offset from the center of the robot

    if (limelightFront == null) {
      System.err.println("Limelight Front failed to initialize.");
    } else {
    limelightFront
        .getSettings()
        .withCameraOffset(
            new Pose3d(
                LimelightConstants.LimelightFront.xOffset,
                LimelightConstants.LimelightFront.yOffset,
                LimelightConstants.LimelightFront.zOffset,
                LimelightConstants.LimelightFront.rotationOffset));
    }
    if (limelightSide == null) {
      System.err.println("Limelight Side failed to initialize.");
    } else {
      limelightSide
          .getSettings()
          .withCameraOffset(
              new Pose3d(
                  LimelightConstants.LimelightSide.xOffset,
                  LimelightConstants.LimelightSide.yOffset,
                  LimelightConstants.LimelightSide.zOffset,
                  LimelightConstants.LimelightSide.rotationOffset));
    }
  }
  @Override
  public void periodic() {
    LimelightHelpers.PoseEstimate side = getBotPoseSide();
    if (side != null && side.tagCount > 0) {
      SmartDashboard.putNumber("Limelight Side Pose/X", side.pose.getX());
      SmartDashboard.putNumber("Limelight Side Pose/Y", side.pose.getY());
    }
  }

  // TAG TARGETTING
  private boolean frontHasTarget() {
    // Using Limelight Helpers, get the TV (Valid Target) value.
    return LimelightHelpers.getTV(LimelightConstants.LimelightFront.name);
  }

  private boolean sideHasTarget() {
    return LimelightHelpers.getTV(LimelightConstants.LimelightSide.name);
  }

  public boolean hasTarget() {
    return frontHasTarget() && sideHasTarget();
  }

  // APRIL TAG ID
  public int getAprilTagId() {
    // Using LimelightHelpers, get the April Tag (fiducial) ID
    return (int) LimelightHelpers.getFiducialID(LimelightConstants.LimelightFront.name);
  }

  // ROBOT POSITION

  /**
   * Returns the closest tag distance. Prefers rawFiducials (per-tag distances), but falls back
   * to avgTagDist when rawFiducials is empty (NT array size mismatch race condition).
   * Returns Double.MAX_VALUE only if the estimate itself is null or has no tags.
   */
  private double getClosestTagDist(LimelightHelpers.PoseEstimate estimate) {
    if (estimate == null || estimate.tagCount == 0) return Double.MAX_VALUE;

    // Try per-tag distances first
    if (estimate.rawFiducials != null && estimate.rawFiducials.length > 0) {
      double min = Double.MAX_VALUE;
      for (LimelightHelpers.RawFiducial f : estimate.rawFiducials) {
        if (f.distToCamera < min) min = f.distToCamera;
      }
      return min;
    }

    // Fallback: avgTagDist is always populated from the NT array directly
    return estimate.avgTagDist;
  }

  /**
   * Chooses MT1 or MT2 based on closest visible tag distance.
   * Close tags (< MT_SWITCH_DISTANCE) → MT1 (full 6DOF, best at close range).
   * Far tags (>= MT_SWITCH_DISTANCE) → MT2 (gyro-constrained, handles ambiguity better).
   * Falls back to MT1 if the chosen estimate has no data.
   */
  private LimelightHelpers.PoseEstimate chooseMegaTag(
      LimelightHelpers.PoseEstimate mt1,
      LimelightHelpers.PoseEstimate mt2,
      String telemetryPrefix) {

    double closestDist = getClosestTagDist(mt1);
    boolean useMT2 = closestDist >= LimelightConstants.MT_SWITCH_DISTANCE_METERS;

    SmartDashboard.putNumber(telemetryPrefix + "/ClosestTagDist", closestDist);
    SmartDashboard.putBoolean(telemetryPrefix + "/UsingMT2", useMT2);

    // If we chose MT2 but it has no data, fall back to MT1
    if (useMT2 && (mt2 == null || mt2.tagCount == 0)) {
      SmartDashboard.putBoolean(telemetryPrefix + "/MT2Fallback", true);
      return mt1;
    }
    SmartDashboard.putBoolean(telemetryPrefix + "/MT2Fallback", false);

    return useMT2 ? mt2 : mt1;
  }

  public LimelightHelpers.PoseEstimate getBotPoseFront() {
    LimelightHelpers.SetRobotOrientation(
        LimelightConstants.LimelightFront.name,
        Odometry.getInstance().getPose().getRotation().getRadians(),
        0.0, 0.0, 0.0, 0.0, 0.0);

    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightFront.name);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LimelightFront.name);

    return chooseMegaTag(mt1, mt2, "Vision/Front");
  }

  public LimelightHelpers.PoseEstimate getBotPoseSide() {
    LimelightHelpers.SetRobotOrientation(
        LimelightConstants.LimelightSide.name,
        Odometry.getInstance().getPose().getRotation().getRadians(),
        0.0, 0.0, 0.0, 0.0, 0.0);

    LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightSide.name);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LimelightSide.name);

    return chooseMegaTag(mt1, mt2, "Vision/Side");
  }
}
