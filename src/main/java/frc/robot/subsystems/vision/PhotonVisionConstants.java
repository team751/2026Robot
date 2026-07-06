package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class PhotonVisionConstants {
  public static final String FRONT_CAMERA_NAME = "front";
  public static final String SIDE_CAMERA_NAME = "side";

  // Same starting values as LimelightConstants — retune independently on real hardware,
  // since PhotonVision's distance/ambiguity behavior won't necessarily match the Limelight's.
  public static final Matrix<N3, N1> FRONT_STD_DEVS = VecBuilder.fill(0.2, 0.2, 99999.0);
  public static final Matrix<N3, N1> SIDE_STD_DEVS = VecBuilder.fill(0.3, 0.3, 99999.0);
  public static final double MULTI_TAG_STD_DEV_FACTOR = 0.3;

  // Rotations are deliberately NOT shared with LimelightConstants: those use Limelight
  // conventions (pitch "up is +", yaw "right is +") while WPILib Rotation3d — what PhotonVision
  // expects — is the opposite (positive pitch = down, positive yaw = left). Re-measure both on
  // real hardware; sim cannot catch a wrong sign because it places and interprets the camera
  // with the same transform.
  public static final Transform3d FRONT_CAMERA_OFFSET =
      new Transform3d(
          new Translation3d(
              LimelightConstants.LimelightFront.xOffset,
              LimelightConstants.LimelightFront.yOffset,
              LimelightConstants.LimelightFront.zOffset), // x, y, and z in meters
          new Rotation3d(0, Units.degreesToRadians(-30), 0)); // tilted 30 deg UP
  public static final Transform3d SIDE_CAMERA_OFFSET =
      new Transform3d(
          new Translation3d(
              LimelightConstants.LimelightSide.xOffset,
              LimelightConstants.LimelightSide.yOffset,
              LimelightConstants.LimelightSide.zOffset), // x, y, and z in meters
          // TODO(hardware): unverified sign — the Limelight comment implies this camera faces
          // LEFT, which in WPILib convention would be +90. Harmless in sim either way.
          new Rotation3d(0, 0, Units.degreesToRadians(-90)));
}
