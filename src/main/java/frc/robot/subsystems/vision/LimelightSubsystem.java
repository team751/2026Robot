// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.LimelightHelpers;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import limelight.Limelight;
/* TODO: Rough Overview of Vision/Limelight
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
  private final Limelight limelightBack;
  
  public static LimelightSubsystem getInstance() {
    if (instance == null) instance = new LimelightSubsystem();
    return instance;
  }

  private LimelightSubsystem() {
    // Inits both limelights using their pre-set names
    limelightFront = new Limelight(LimelightConstants.LimelightFront.name);

    limelightBack = new Limelight(LimelightConstants.LimelightBack.name);

    // Sets the settings for each limelight with their offset from the center of the robot
    limelightFront.getSettings()
      .withCameraOffset(new Pose3d(
        LimelightConstants.LimelightFront.xOffset.in(Units.Meters),
        LimelightConstants.LimelightFront.yOffset.in(Units.Meters),
        LimelightConstants.LimelightFront.zOffset.in(Units.Meters),
        LimelightConstants.LimelightFront.rotationOffset)
    );

    limelightBack.getSettings()
      .withCameraOffset(new Pose3d(
        LimelightConstants.LimelightBack.xOffset.in(Units.Meters),
        LimelightConstants.LimelightBack.yOffset.in(Units.Meters),
        LimelightConstants.LimelightBack.zOffset.in(Units.Meters),
        LimelightConstants.LimelightBack.rotationOffset)
        );
  }

  // TAG TARGETTING
  private boolean frontHasTarget() {
    // Using Limelight Helpers, get the TV (Valid Target) value.
    return LimelightHelpers.getTV(LimelightConstants.LimelightFront.name);
  }

  private boolean backHasTarget() {
    return LimelightHelpers.getTV(LimelightConstants.LimelightBack.name);
  }

  public boolean hasTarget() {
    return frontHasTarget() && backHasTarget();
  }


  // APRIL TAG ID
  public int getAprilTagId() {
    // Using LimelightHelpers, get the April Tag (fiducial) ID
    return (int) LimelightHelpers.getFiducialID(LimelightConstants.LimelightFront.name);
  }


  // ROBOT POSITION
  public Pose2d getBotPoseFront() {
    // Sets the robot orientation before getting the robot position
    LimelightHelpers.SetRobotOrientation(LimelightConstants.LimelightBack.name, drive.getRotation3d().getZ(), 0.0, 0.0, 0.0, 0.0, 0.0);
    
    // Gets the current side (alliance)
    var alliance = DriverStation.getAlliance();

    // If the alliance is red, get the robot position in the Red Alliance coordinates
    // Otherwise get the robot position in the Blue Alliance coordinates
    if (alliance.get() == DriverStation.Alliance.Red) {
      if (!(LimelightHelpers.getBotPoseEstimate_wpiRed(LimelightConstants.LimelightFront.name).pose.getX() == 0 && LimelightHelpers.getBotPoseEstimate_wpiRed(LimelightConstants.LimelightFront.name).pose.getY() == 0)) {
        return LimelightHelpers.getBotPoseEstimate_wpiRed(LimelightConstants.LimelightFront.name).pose;
      } else {
        return null;
      }
    } else {
      if (!(LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightFront.name).pose.getX() == 0 && LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightFront.name).pose.getY() == 0)) {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightFront.name).pose;
      } else {
        return null;
      }
    }
  }

  public Pose2d getBotPoseBack() {
    LimelightHelpers.SetRobotOrientation(LimelightConstants.LimelightBack.name, drive.getRotation3d().getZ(), 0.0, 0.0, 0.0, 0.0, 0.0);
    var alliance = DriverStation.getAlliance();

    if (alliance.get() == DriverStation.Alliance.Red) {
      if (!(LimelightHelpers.getBotPoseEstimate_wpiRed(LimelightConstants.LimelightBack.name).pose.getX() == 0 && LimelightHelpers.getBotPoseEstimate_wpiRed(LimelightConstants.LimelightBack.name).pose.getY() == 0)) {
        return LimelightHelpers.getBotPoseEstimate_wpiRed(LimelightConstants.LimelightBack.name).pose;
      } else {
        return null;
      }
    } else {
      if (!(LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightBack.name).pose.getX() == 0 && LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightBack.name).pose.getY() == 0)) {
        return LimelightHelpers.getBotPoseEstimate_wpiBlue(LimelightConstants.LimelightBack.name).pose;
      } else {
        return null;
      }
    }
  }

  // Interpolates (averages) the front and back camera positions
  public Pose2d getBotPoseInterpolated() {
    if (backHasTarget() && frontHasTarget()) {
      // If both cameras have a target, interpolate using the front camera robot pose as the base
      return this.getBotPoseFront().interpolate(getBotPoseBack(), 0.5);
    } else if (backHasTarget()) {
      // If the only the back camera has a target, return the back camera's robot position
      return this.getBotPoseBack();
    } else {
      // If only the front camera has a target, return the front camera's robot position
       return this.getBotPoseFront();
    }
  }

  // INIT
  public void robotInit() {
    // Sets up a camera server to display the two limelights stream on SmartDashboard (or Elastic if ur using that)
    String limelightFrontUrl = LimelightConstants.LimelightFront.streamIp;
    HttpCamera limelightFrontCam = new HttpCamera("Limelight", limelightFrontUrl);
    CameraServer.startAutomaticCapture(limelightFrontCam);

    String limelightBackUrl = LimelightConstants.LimelightBack.streamIp;
    HttpCamera limelightBackCam = new HttpCamera("Limelight 2", limelightBackUrl);
    CameraServer.startAutomaticCapture(limelightBackCam);
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}
}
