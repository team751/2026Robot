package frc.robot.subsystems.vision;

import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;

public class VisionConstants {
	public static class Limelight {
		public static final String version = "3G";
		public static final String streamIp = "http://10.7.51.11:5800";
		public static final String dashboardIp = "http://10.7.51.11:5801";
		public static final String name = "limelight";

		public static final Distance zOffset = Units.Inches.of(12.224 + 3.75); // inches
		public static final Distance yOffset = Units.Inches.of(13 - 6.01); // inches
	}
}
