// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;
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
  private final SwerveSubsystem drive = SwerveSubsystem.getInstance();

  private static LimelightSubsystem instance;
  private final Limelight limelightFront;
  private final Limelight limelightSide;

  public static LimelightSubsystem getInstance() {
    if (instance == null) instance = new LimelightSubsystem();
    return instance;
  }

  private LimelightSubsystem() {
    // Inits both limelights using their pre-set names
    limelightFront = new Limelight(LimelightConstants.LimelightFront.name);

    limelightSide = new Limelight(LimelightConstants.LimelightSide.name);

    // Sets the settings for each limelight with their offset from the center of the robot
    limelightFront
        .getSettings()
        .withCameraOffset(
            new Pose3d(
                LimelightConstants.LimelightFront.xOffset,
                LimelightConstants.LimelightFront.yOffset,
                LimelightConstants.LimelightFront.zOffset,
                LimelightConstants.LimelightFront.rotationOffset));

    limelightSide
        .getSettings()
        .withCameraOffset(
            new Pose3d(
                LimelightConstants.LimelightSide.xOffset,
                LimelightConstants.LimelightSide.yOffset,
                LimelightConstants.LimelightSide.zOffset,
                LimelightConstants.LimelightSide.rotationOffset));
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
        Units.radiansToDegrees(drive.getRotation3d().getZ()),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0);

    // If theres no april tag seen return null
    if (LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LimelightFront.name).pose == null
         && LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LimelightFront.name).pose == null
         && LimelightHelpers.getTA(LimelightConstants.LimelightFront.name) < 50) {
      return null;
    }

    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LimelightFront.name).pose;
    // if (LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightFront.name).pose
    // == null
    //     &&
    // LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightFront.name).pose ==
    // null) {
    //   return null;
    // }

    // return
    // LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightFront.name).pose;
  }

  public Pose2d getBotPoseSide() {
    // Sets the robot orientation before getting the robot position
    LimelightHelpers.SetRobotOrientation(
        LimelightConstants.LimelightSide.name,
        Units.radiansToDegrees(drive.getRotation3d().getZ()),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0);

    // If theres no april tag seen return null
    if (LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LimelightSide.name).pose == null
        && LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LimelightSide.name).pose == null
        && LimelightHelpers.getTA(LimelightConstants.LimelightFront.name) < 50) {
      return null;
    }
    return LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimelightConstants.LimelightSide.name).pose;
    // if (LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightSide.name).pose
    // == null
    //     &&
    // LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightSide.name).pose ==
    // null) {
    //   return null;
    // }
    // return
    // LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightSide.name).pose;
  }
}
