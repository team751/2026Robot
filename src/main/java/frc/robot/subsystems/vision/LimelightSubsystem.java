// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
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
  private final Limelight limelightFront;
  private final Limelight limelightSide;

  public static LimelightSubsystem getInstance() {
    if (instance == null) instance = new LimelightSubsystem();
    return instance;
  }

  private LimelightSubsystem() {
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
    if (getBotPoseSide() != null) {
      SmartDashboard.putNumber("Limelight Side Pose/X", getBotPoseSide().getX());
      SmartDashboard.putNumber("Limelight Side Pose/Y", getBotPoseSide().getY());
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
  public Pose2d getBotPoseFront() {
    // Sets the robot orientation before getting the robot position
    LimelightHelpers.SetRobotOrientation(
        LimelightConstants.LimelightFront.name,
        Odometry.getInstance().getPose().getRotation().getRadians(),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0);

        Pose2d frontPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightFront.name).pose;

    // If theres no april tag seen return null
    if (frontPoseEstimate == null) {
      return null;
    }

    //return frontPoseEstimate;
    if (LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightFront.name).pose
    == null
        &&
    LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightFront.name).pose ==
    null) {
      return null;
    }

    return
    LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightFront.name).pose;
  }

  public Pose2d getBotPoseSide() {
    // Sets the robot orientation before getting the robot position
    LimelightHelpers.SetRobotOrientation(
        LimelightConstants.LimelightSide.name,
        Odometry.getInstance().getPose().getRotation().getRadians(),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0);

        Pose2d sidePoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightSide.name).pose;

    // If theres no april tag seen return null
    if (sidePoseEstimate == null) {
      return null;
    }
    //return sidePoseEstimate;
    if (LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightSide.name).pose
    == null
        &&
    LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightSide.name).pose ==
    null) {
      return null;
    }
    return
    LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightSide.name).pose;
  }
}
