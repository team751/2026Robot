package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.Units;

/* LimelightConstants.java Info
 * This is where all the basic, and often/literally never
 * changed info for Limelight is stored.
 *
 * So things like the Limeligh name or offset from the center of the robot.
 * The name is almost never changed, but gets used more often than
 * you think.
 *
 * The offset needs to be tweaked a lot, and having to change it in 3 different places
 * is annoying, so having it as a variable here is much easier.
 */

public class LimelightConstants {
  // --- Vision Fusion ---

  // Per-camera base std devs (x, y, theta). Theta=99999 means heading comes only from gyro.
  // Lower values = more trust. The 3G has a better sensor/wider FOV, so it gets lower base values.
  public static final Matrix<N3, N1> FRONT_STD_DEVS = VecBuilder.fill(0.2, 0.2, 99999.0); // 3G
  public static final Matrix<N3, N1> SIDE_STD_DEVS = VecBuilder.fill(0.3, 0.3, 99999.0);    // 3

  // When multiple tags are visible, the solution is more constrained. Multiply std devs by this factor.
  // Lower = more trust in multi-tag. 1.0 = no bonus.
  public static final double MULTI_TAG_STD_DEV_FACTOR = 0.5;

  // Reject vision entirely when the robot is spinning faster than this (deg/s).
  // Fast rotation causes gyro-to-vision latency mismatch that corrupts MegaTag2.
  public static final double MAX_ANGULAR_VELOCITY_DPS = 720.0;

  // MegaTag switching: use MT1 when closest tag is nearer than this (meters), MT2 when farther.
  // MT1 is more accurate at close range (full 6DOF solve). MT2 uses gyro-constrained solve,
  // which is better at distance where single-tag ambiguity makes MT1 unreliable.
  public static final double MT_SWITCH_DISTANCE_METERS = 1.1;

  // --- Pose Stability ---

  // Vision must agree with odometry within this distance (meters) to count as stable
  public static final double POSE_STABLE_EPSILON_METERS = 0.10;
  // Number of consecutive agreeing updates before pose is considered stable
  public static final int POSE_STABLE_THRESHOLD = 10;

  public static class LimelightFront {
    public static final String version = "3G";
    public static final String streamIp = "http://10.7.51.71:5800";
    public static final String shuffleStreamIp = "mjpeg:http://10.7.51.71:5800";
    public static final String dashboardIp = "http://10.7.51.71:5801";
    public static final String name = "limelight-front";
    public static final double pipelineLatencySeconds = 0.0;

    // change the offset
    public static final double zOffset =
        Units.Meters.convertFrom(41.2768, Units.Centimeters); // Up/Down (+ up, - down)
    public static final double yOffset =
        Units.Meters.convertFrom(-14.5331, Units.Centimeters); // Left/Right (- left, +right)
    public static final double xOffset =
        Units.Meters.convertFrom(4.1568, Units.Centimeters); // Forward/Backward (+ forward, - backward)

    public static final Rotation3d rotationOffset =
        new Rotation3d(
            0,
            Units.Radians.convertFrom(30,Units.Degrees), // Up is + and down is - (Degrees)
            0); // Left is - and right is + (degrees)
  }

  public static class LimelightSide {
    public static final String version = "3";
    public static final String streamIp = "http://10.7.51.75:5800";
    public static final String shuffleStreamIp = "mjpeg:http://10.7.51.75:5800";
    public static final String dashboardIp = "http://10.7.51.75:5801";
    public static final String name = "limelight-side";
    public static final double pipelineLatencySeconds = 0.0;

    public static final double zOffset =
        Units.Meters.convertFrom(29.06, Units.Centimeters); // Up/Down (+ up, - down)
    public static final double yOffset =
        Units.Meters.convertFrom(11.19, Units.Centimeters); // Left/Right (- left, + right)
    public static final double xOffset =
        Units.Meters.convertFrom(
            -25.39, Units.Centimeters); // Forward/Backward (+ forward, - backward)

    public static final Rotation3d rotationOffset =
        new Rotation3d(
            0,
            0,
            Units.Radians.convertFrom(-90, Units.Degrees)); // Left is - and right is + (degrees)
  }
}
