package frc.robot.subsystems.vision;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class LimelightConstants {
    public static class Limelight {
        public static final String version = "3G";
        public static final String steamIp = "http://10.7.51.200:5800";
        public static final String shuffleSteamIp = "mjpeg:http://10.7.51.200.5800";
        public static final String dashboardIp = "http://10.7.51.200:5801";
        public static final String name = "limelight";
        public static final double pipelineLatencySeconds = 0.0;
        
        public static final Distance zOffset = Units.Inches.of(12.224 + 3.75);
        public static final Distance yOffset = Units.Inches.of(13 - 6.01);
        public static final Distance xOffset = Units.Inches.of(0);
    }
}
