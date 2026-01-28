package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

/* TODO: LimelightConstants.java Info
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
        public static final String version = "3G";
        public static final String streamIp = "http://10.7.51.71:5800";
        // For ShuffleBoard. Hopefully this is never used :D
        public static final String shuffleStreamIp = "mjpeg:http://10.7.51.71:5800";
        public static final String dashboardIp = "http://10.7.51.71:5801";
        public static final String name = "limelight-front";
        public static final double pipelineLatencySeconds = 0.0;
    
        public static final Distance zOffset = Units.Meters.of(0.235); // Up/Down
        public static final Distance yOffset = Units.Meters.of(0.025); // Left/Right
        public static final Distance xOffset = Units.Meters.of(-0.225); // Forward/Backward - Seems like for drivebase, going to the front is negative
        // Front Limelight doesn't need an offset since.. its.. like already facing forward??
        public static final Rotation3d rotationOffset = new Rotation3d(0,0,0);
    }

    public static class LimelightBack {
        public static final String version = "2";
        public static final String streamIp = "http://10.7.51.75:5800";
        public static final String shuffleStreamIp = "mjpeg:http://10.7.51.75:5800";
        public static final String dashboardIp = "http://10.7.51.75:5801";
        public static final String name = "limelight-back";
        public static final double pipelineLatencySeconds = 0.0;

        public static final Distance zOffset = Units.Meters.of(0.3); // Up/Down
        public static final Distance yOffset = Units.Meters.of(0.025); // Left/Right
        public static final Distance xOffset = Units.Meters.of(-0.135); // Forward/Backward
        // Back Limelight needs an offset of 180 :)
        public static final Rotation3d rotationOffset = new Rotation3d(0,0,Units.Radians.convertFrom(180, Units.Degrees));
    }
}
