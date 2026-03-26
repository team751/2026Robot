package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
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
  public static class LimelightFront {
    public static final String version = "3";
    public static final String streamIp = "http://10.7.51.71:5800";
    public static final String shuffleStreamIp = "mjpeg:http://10.7.51.71:5800";
    public static final String dashboardIp = "http://10.7.51.71:5801";
    public static final String name = "limelight-front";
    public static final double pipelineLatencySeconds = 0.0;

    // change the offset
    public static final double zOffset =
        Units.Meters.convertFrom(41.2768, Units.Centimeters); // Up/Down (+ up, - down)
    public static final double yOffset =
        Units.Meters.convertFrom(-14.5331, Units.Centimeters); // Left/Right (+ left, -right)
    public static final double xOffset =
        Units.Meters.convertFrom(-4.1568, Units.Centimeters); // Forward/Backward (- forward, + backward)

    public static final Rotation3d rotationOffset =
        new Rotation3d(
            0,
            Units.Radians.convertFrom(30,Units.Degrees), // Up is + and down is - (Degrees)
            0); // Left is - and right is + (degrees)
  }

  public static class LimelightSide {
    public static final String version = "2";
    public static final String streamIp = "http://10.7.51.75:5800";
    public static final String shuffleStreamIp = "mjpeg:http://10.7.51.75:5800";
    public static final String dashboardIp = "http://10.7.51.75:5801";
    public static final String name = "limelight-side";
    public static final double pipelineLatencySeconds = 0.0;

    public static final double zOffset =
        Units.Meters.convertFrom(29.06, Units.Centimeters); // Up/Down (+ up, - down)
    public static final double yOffset =
        Units.Meters.convertFrom(11.19, Units.Centimeters); // Left/Right (+ left, -right)
    public static final double xOffset =
        Units.Meters.convertFrom(
            -25.39, Units.Centimeters); // Forward/Backward (- forward, + backward)

    public static final Rotation3d rotationOffset =
        new Rotation3d(
            0,
            0,
            Units.Radians.convertFrom(0, Units.Degrees)); // Left is - and right is + (degrees)
  }
}
