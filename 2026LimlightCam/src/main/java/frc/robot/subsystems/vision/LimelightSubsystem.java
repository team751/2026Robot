// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import limelight.Limelight;
import limelight.networktables.LimelightResults;

public class LimelightSubsystem extends SubsystemBase {

  private NetworkTable limelightFrontTable = NetworkTableInstance.getDefault().getTable("limelight-front");
  private NetworkTable limelightBackTable = NetworkTableInstance.getDefault().getTable("limelight-back");
 
  private static LimelightSubsystem instance;
  private final Limelight limelightFront;
  private final Limelight limelightBack;
  
  public static LimelightSubsystem getInstance() {
    if (instance == null) instance = new LimelightSubsystem();
    return instance;
  }

  private LimelightSubsystem() {
    limelightFront = new Limelight(LimelightConstants.LimelightFront.name);

    limelightBack = new Limelight(LimelightConstants.LimelightBack.name);

    limelightFront.getSettings()
      .withCameraOffset(new Pose3d(
        LimelightConstants.LimelightFront.xOffset.in(Units.Meters),
        LimelightConstants.LimelightFront.yOffset.in(Units.Meters),
        LimelightConstants.LimelightFront.zOffset.in(Units.Meters),
        new Rotation3d())
    );

    limelightBack.getSettings()
      .withCameraOffset(new Pose3d(
        LimelightConstants.LimelightBack.xOffset.in(Units.Meters),
        LimelightConstants.LimelightBack.yOffset.in(Units.Meters),
        LimelightConstants.LimelightBack.zOffset.in(Units.Meters),
        new Rotation3d())
        );
  }

  public boolean hasTarget() {
    double tv = limelightFrontTable.getEntry("tv").getDouble(0.0);

    if (tv < 1.0) {
      return false;
    }

    return true;
  }


  public Optional<Pose2d> getEstimatedPose() {
    return limelightFront.getData().getResults().map(LimelightResults::getBotPose2d);
  }


  public int getAprilTagId() {
    NetworkTableEntry tidEntry = limelightFrontTable.getEntry("tid");

    
    double tid = tidEntry.getDouble(Double.NaN);
    if (!Double.isNaN(tid) && tid >= 0) {
      return (int) tid;
    }

    double[] tidArray = limelightFrontTable.getEntry("tid").getDoubleArray(new double[0]);
    if (tidArray.length > 0) {
      return (int) tidArray[0];
    }

    double t0 = limelightFrontTable.getEntry("tid0").getDouble(Double.NaN);
    if (!Double.isNaN(t0)) {
      return (int) t0;
    }

    return -1;
  }

  public Pose2d getBotPoseFront() {
    double[] botpose = limelightFrontTable.getEntry("botpose_orb").getDoubleArray(new double[0]);

    return new Pose2d(botpose[0],botpose[1],new Rotation2d(Units.Radians.convertFrom(botpose[5],Units.Degrees)));
  }

  public Pose2d getBotPoseBack() {
    double[] botpose = limelightFrontTable.getEntry("botpose_orb").getDoubleArray(new double[0]);

    return new Pose2d(botpose[0], botpose[1], new Rotation2d(Units.Radians.convertFrom(botpose[5], Units.Degrees)));
  }

  private Pose2d combineBotPose() {
    return this.getBotPoseFront().relativeTo(getBotPoseBack());
  }

  public Pose2d getBotPoseFull() {
    return this.combineBotPose();
  }

  public Pose2d getBotPosePt2() {
    return this.getBotPoseFront().interpolate(getBotPoseBack(), 0.5);
  }



  public void robotInit() {
    String limelightFrontUrl = LimelightConstants.LimelightFront.streamIp;
    HttpCamera limelightFrontCam = new HttpCamera("Limelight", limelightFrontUrl);
    CameraServer.startAutomaticCapture(limelightFrontCam);

    String limelightBackUrl = LimelightConstants.LimelightBack.streamIp;
    HttpCamera limelightBackCam = new HttpCamera("Limelight 2", limelightBackUrl);
    CameraServer.startAutomaticCapture(limelightBackCam);
  }

  // private double lastPrintTime = 0.0;
  @Override
  public void periodic() {

    // double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    // if (now - lastPrintTime >= 0.5) {
    //   lastPrintTime = now;

    //   System.out.println(this.getBotPose());


    // }
  }

  @Override
  public void simulationPeriodic() {}
}
