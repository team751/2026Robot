package frc.robot.subsystems.vision;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class LimelightConstants {
    public static class LimelightFront {
        public static final String version = "3G";
        public static final String streamIp = "http://10.7.51.71:5800";
        public static final String shuffleStreamIp = "mjpeg:http://10.7.51.71:5800";
        public static final String dashboardIp = "http://10.7.51.71:5801";
        public static final String name = "limelight-front";
        public static final double pipelineLatencySeconds = 0.0;
        
        public static final Distance zOffset = Units.Meters.of(0.07);
        public static final Distance yOffset = Units.Meters.of(0.07);
        public static final Distance xOffset = Units.Meters.of(0.025);
    }

    public static class LimelightBack {
        public static final String version = "2";
        public static final String streamIp = "http://10.7.51.75:5800";
        public static final String shuffleStreamIp = "mjpeg:http://10.7.51.75:5800";
        public static final String dashboardIp = "http://10.7.51.75:5801";
        public static final String name = "limelight-back";
        public static final double pipelineLatencySeconds = 0.0;

        public static final Distance zOffset = Units.Meters.of(0.0);
        public static final Distance yOffset = Units.Meters.of(0.09);
        public static final Distance xOffset = Units.Meters.of(0.025);
    }
}
