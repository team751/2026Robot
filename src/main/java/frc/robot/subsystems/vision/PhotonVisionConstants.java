package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public class PhotonVisionConstants {
  public static final String FRONT_CAMERA_NAME = "front";
  public static final String SIDE_CAMERA_NAME = "side";

  public static final Transform3d FRONT_CAMERA_OFFSET =
      new Transform3d(
          new Translation3d(
              LimelightConstants.LimelightFront.xOffset,
              LimelightConstants.LimelightFront.yOffset,
              LimelightConstants.LimelightFront.zOffset), // x, y, and z in meters
          LimelightConstants.LimelightFront.rotationOffset);
  public static final Transform3d SIDE_CAMERA_OFFSET =
      new Transform3d(
          new Translation3d(
              LimelightConstants.LimelightSide.xOffset,
              LimelightConstants.LimelightSide.yOffset,
              LimelightConstants.LimelightSide.zOffset), // x, y, and z in meters
          LimelightConstants.LimelightSide.rotationOffset);
}
