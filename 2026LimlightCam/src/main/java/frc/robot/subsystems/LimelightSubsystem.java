// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.Optional;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import limelight.Limelight;
import limelight.networktables.LimelightResults;

public class LimelightSubsystem extends SubsystemBase {

  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  

  private static LimelightSubsystem instance;
  private final Limelight limelight;

  private final Field2d field = new Field2d();
  
  public static LimelightSubsystem getInstance() {
    if (instance == null) instance = new LimelightSubsystem();
    return instance;
  }

  private LimelightSubsystem() {
    limelight = new Limelight(LimelightConstants.Limelight.name);

    limelight.getSettings()
      .withCameraOffset(new Pose3d(
        0,
        LimelightConstants.Limelight.yOffset.in(Units.Meters),
        LimelightConstants.Limelight.zOffset.in(Units.Meters),
        new Rotation3d())
    );

    SmartDashboard.putData("Limelight Field", field);
  }

  public boolean hasTarget() {
    double tv = table.getEntry("tv").getDouble(0.0);

    if (tv < 1.0) {
      return false;
    }

    return true;
  }


  public Optional<Pose2d> getEstimatedPose() {
    return limelight.getData().getResults().map(LimelightResults::getBotPose2d);
  }


  public int getAprilTagId() {
    NetworkTableEntry tidEntry = table.getEntry("tid");

    
    double tid = tidEntry.getDouble(Double.NaN);
    if (!Double.isNaN(tid) && tid >= 0) {
      return (int) tid;
    }

    double[] tidArray = table.getEntry("tid").getDoubleArray(new double[0]);
    if (tidArray.length > 0) {
      return (int) tidArray[0];
    }

    double t0 = table.getEntry("tid0").getDouble(Double.NaN);
    if (!Double.isNaN(t0)) {
      return (int) t0;
    }

    return -1;
  }

  private double lastPrintTime = 0.0;
  private double[] bob = new double[0];
  private String botPoseString = "[";

  public void robotInit() {
    String limelightUrl = "http://10.7.51.200:5800";
    HttpCamera limlightCam = new HttpCamera("Limelight", limelightUrl);
    CameraServer.startAutomaticCapture(limlightCam);
  }

  @Override
  public void periodic() {

    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (now - lastPrintTime >= 0.5) {
      lastPrintTime = now;
      double[] limelightBotPose = table.getEntry("botpose").getDoubleArray(bob);
      for (int i = 0; i < limelightBotPose.length; i++) {
        botPoseString += limelightBotPose[i] + ", ";
      }
      botPoseString += "]";

      //System.out.println("Robot: " + botPoseString);
      

      System.out.println(this.hasTarget());
      System.out.println(this.getAprilTagId());

      botPoseString = "";
    }
  }

  @Override
  public void simulationPeriodic() {}
}
