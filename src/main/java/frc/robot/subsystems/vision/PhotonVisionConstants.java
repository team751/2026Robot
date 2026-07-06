package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class PhotonVisionConstants {
  public static final String FRONT_CAMERA_NAME = "front";
  public static final String SIDE_CAMERA_NAME = "side";

  // Same starting values as LimelightConstants — retune independently on real hardware,
  // since PhotonVision's distance/ambiguity behavior won't necessarily match the Limelight's.
  public static final Matrix<N3, N1> FRONT_STD_DEVS = VecBuilder.fill(0.2, 0.2, 99999.0);
  public static final Matrix<N3, N1> SIDE_STD_DEVS = VecBuilder.fill(0.3, 0.3, 99999.0);
  public static final double MULTI_TAG_STD_DEV_FACTOR = 0.3;

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
