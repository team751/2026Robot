package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import frc.robot.subsystems.drive.generated.TunerConstants;

/**
 * Constants derived from (or built alongside) the Tuner X-generated {@link TunerConstants}, plus
 * the {@link RobotConfig} PathPlanner needs for autonomous path following. Unlike TunerConstants,
 * this file is hand-written and safe to edit.
 */
public class SwerveConstants {
  // Top theoretical speed at 12V, taken straight from the Tuner X module gearing/wheel config.
  public static final double maxSpeed = TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond);
  public static final double maxAngularSpeed =
      Units.RotationsPerSecond.of(1.1).in(Units.RadiansPerSecond);

  /** PathPlanner's PID/motion-profile tuning for autonomous path following (not teleop). */
  public static class AutoConstants {
    public static double kMaxSpeedMetersPerSecond = 7; // 3
    public static double kMaxAccelerationMetersPerSecondSquared = 5.5; // 3
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

  // Physical description of one swerve module (wheel radius, drive motor, gear ratio, etc)
  // that PathPlanner uses to simulate/plan feasible paths.
  private static ModuleConfig moduleConfig =
      new ModuleConfig(0.05, 10, 1.0, DCMotor.getKrakenX60Foc(1), 30400, 1);
  // FL, FR, BL, BR module positions relative to robot center (meters) — must match the
  // physical layout, same ordering TunerConstants/SwerveSubsystem use elsewhere.
  private static Translation2d[] moduleOffsets =
      new Translation2d[] {
        new Translation2d(-0.263525, 0.263525),
        new Translation2d(0.263525, 0.263525),
        new Translation2d(-0.263525, -0.263525),
        new Translation2d(0.263525, -0.263525)
      };
  // Robot mass (kg) and MOI (kg·m^2), used by PathPlanner's path planner + AutoBuilder.
  public static RobotConfig robotConfig = new RobotConfig(50, 5, moduleConfig, moduleOffsets);
}
