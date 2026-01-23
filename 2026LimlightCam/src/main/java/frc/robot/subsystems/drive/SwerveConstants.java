package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
// import com.pathplanner.lib.config.ModuleConfig;
// import com.pathplanner.lib.config.RobotConfig;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.drive.generated.TunerConstants;

public class SwerveConstants {
	public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
	public static final double maxAngularSpeed =
			Units.RotationsPerSecond.of(1.1).in(Units.RadiansPerSecond);

	public static class AutoConstants {
		public static double kMaxSpeedMetersPerSecond = 7; // 3
		public static double kMaxAccelerationMetersPerSecondSquared = 2.5; // 3
		public static double kMaxAngularSpeedRadiansPerSecond = 4.2 * Math.PI;
		public static double kMaxAngularSpeedRadiansPerSecondSquared = 4 * Math.PI;

		// especially these values
		public static double kPXController = 0.6;
		public static double kPYController = 0.6;
		public static double kPThetaController = 4;

		/* Constraint for the motion profiled robot angle controller */
		public static TrapezoidProfile.Constraints kThetaControllerConstraints =
				new TrapezoidProfile.Constraints(
						kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

		public static TrapezoidProfile.Constraints kXControllerConstraints =
				new TrapezoidProfile.Constraints(
						kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);

		public static TrapezoidProfile.Constraints kYControllerConstraints =
				new TrapezoidProfile.Constraints(
						kMaxSpeedMetersPerSecond, kMaxAccelerationMetersPerSecondSquared);
	}

	
	private static ModuleConfig moduleConfig =
			new ModuleConfig(0.05, 10, 5, DCMotor.getKrakenX60Foc(1), 30400, 1);
	// public static RobotConfig robotConfig = new RobotConfig(55, 5, moduleConfig, 0.8);
	private static Translation2d[] moduleOffsets =
			new Translation2d[] {
				new Translation2d(-0.263525, 0.263525),
				new Translation2d(0.263525, 0.263525),
				new Translation2d(-0.263525, -0.263525),
				new Translation2d(0.263525, -0.263525)
			};
	public static RobotConfig robotConfig = new RobotConfig(50, 5, moduleConfig, moduleOffsets);
}
