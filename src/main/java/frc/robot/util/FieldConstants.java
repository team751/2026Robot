package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Field element positions and dimensions for the 2026 Rebuilt game.
 *
 * <p>All coordinates use the WPILib field coordinate system (meters):
 * <ul>
 *   <li>Origin (0, 0) at the bottom-right corner of the BLUE alliance wall
 *   <li>+X toward the RED wall
 *   <li>+Y to the left (from BLUE driver perspective)
 * </ul>
 */
public class FieldConstants {

  // ---- Field ----
  public static final double FIELD_LENGTH = 16.540988;
  public static final double FIELD_WIDTH = 8.052;

  // ---- Hub dimensions ----
  public static final double HUB_WIDTH = Units.inchesToMeters(47.0);
  public static final double HUB_HALF_WIDTH = 0.5969; // GoalRadius from RebuiltHub
  public static final double HUB_HEIGHT = Units.inchesToMeters(72.0); // includes catcher
  public static final double HUB_INNER_WIDTH = Units.inchesToMeters(41.7);
  public static final double HUB_INNER_HEIGHT = Units.inchesToMeters(56.5);

  // ---- Hub 3D reference points ----
  public static final Translation3d BLUE_HUB_TOP_CENTER =
      new Translation3d(4.5974, 4.034536, HUB_HEIGHT);
  public static final Translation3d BLUE_HUB_INNER_CENTER =
      new Translation3d(4.5974, 4.034536, HUB_INNER_HEIGHT);
  public static final Translation3d RED_HUB_TOP_CENTER =
      new Translation3d(11.938, 4.034536, HUB_HEIGHT);
  public static final Translation3d RED_HUB_INNER_CENTER =
      new Translation3d(11.938, 4.034536, HUB_INNER_HEIGHT);

  // ---- Hub floor corners (near = toward that alliance's wall) ----
  public static final Translation2d BLUE_HUB_NEAR_LEFT =
      new Translation2d(4.5974 - HUB_HALF_WIDTH, 4.034536 + HUB_HALF_WIDTH);
  public static final Translation2d BLUE_HUB_NEAR_RIGHT =
      new Translation2d(4.5974 - HUB_HALF_WIDTH, 4.034536 - HUB_HALF_WIDTH);
  public static final Translation2d BLUE_HUB_FAR_LEFT =
      new Translation2d(4.5974 + HUB_HALF_WIDTH, 4.034536 + HUB_HALF_WIDTH);
  public static final Translation2d BLUE_HUB_FAR_RIGHT =
      new Translation2d(4.5974 + HUB_HALF_WIDTH, 4.034536 - HUB_HALF_WIDTH);

  public static final Translation2d RED_HUB_NEAR_LEFT =
      new Translation2d(11.938 + HUB_HALF_WIDTH, 4.034536 + HUB_HALF_WIDTH);
  public static final Translation2d RED_HUB_NEAR_RIGHT =
      new Translation2d(11.938 + HUB_HALF_WIDTH, 4.034536 - HUB_HALF_WIDTH);
  public static final Translation2d RED_HUB_FAR_LEFT =
      new Translation2d(11.938 - HUB_HALF_WIDTH, 4.034536 + HUB_HALF_WIDTH);
  public static final Translation2d RED_HUB_FAR_RIGHT =
      new Translation2d(11.938 - HUB_HALF_WIDTH, 4.034536 - HUB_HALF_WIDTH);

  // ---- Tower dimensions ----
  public static final double TOWER_WIDTH = Units.inchesToMeters(49.25);
  public static final double TOWER_DEPTH = Units.inchesToMeters(45.0);
  public static final double TOWER_HEIGHT = Units.inchesToMeters(78.25);
  public static final double TOWER_INNER_OPENING_WIDTH = Units.inchesToMeters(32.250);
  public static final double TOWER_UPRIGHT_HEIGHT = Units.inchesToMeters(72.1);
  public static final double TOWER_LOW_RUNG = Units.inchesToMeters(27.0);
  public static final double TOWER_MID_RUNG = Units.inchesToMeters(45.0);
  public static final double TOWER_HIGH_RUNG = Units.inchesToMeters(63.0);

  // ---- Trench dimensions ----
  public static final double TRENCH_WALL_LENGTH = Units.inchesToMeters(53.0);
  public static final double TRENCH_WALL_WIDTH = Units.inchesToMeters(12.0);
  public static final double TRENCH_HEIGHT = Units.inchesToMeters(47.0);

  // ---- Outpost dimensions ----
  public static final double OUTPOST_WIDTH = Units.inchesToMeters(31.8);
  public static final double OUTPOST_OPENING_HEIGHT = Units.inchesToMeters(28.1);
  public static final double OUTPOST_HEIGHT = Units.inchesToMeters(7.0);

  // ---- Game Elements ----
  public enum ElementType {
    HUB,
    TOWER,
    TRENCH,
    OUTPOST
  }

  public enum GameElement {

    // ----- HUBS -----

    /** Blue hub — scoring target on the blue alliance side. */
    HUB_BLUE(new Pose2d(4.5974, 4.034536, Rotation2d.fromDegrees(0)), true, ElementType.HUB),

    /** Red hub — scoring target on the red alliance side. */
    HUB_RED(new Pose2d(11.938, 4.034536, Rotation2d.fromDegrees(180)), false, ElementType.HUB),

    // ----- TOWERS -----

    /** Blue climbing tower (near the blue alliance wall). */
    TOWER_BLUE(new Pose2d(Units.inchesToMeters(42), Units.inchesToMeters(159), Rotation2d.fromDegrees(0)), true, ElementType.TOWER),

    /** Red climbing tower (near the red alliance wall). */
    TOWER_RED(new Pose2d(Units.inchesToMeters(609), Units.inchesToMeters(170), Rotation2d.fromDegrees(180)), false, ElementType.TOWER),

	// ----- TRENCHES -----

    /** Blue right trench (lower Y, blue side of field). */
    TRENCH_BLUE_RIGHT(new Pose2d(4.6251, 1.4315, Rotation2d.fromDegrees(90)), true, ElementType.TRENCH),

    /** Blue left trench (higher Y, blue side of field). */
    TRENCH_BLUE_LEFT(new Pose2d(4.6251, 6.6385, Rotation2d.fromDegrees(-90)), true, ElementType.TRENCH),

    /** Red right trench (lower Y, red side of field). */
    TRENCH_RED_RIGHT(new Pose2d(11.9149, 1.4315, Rotation2d.fromDegrees(90)), false, ElementType.TRENCH),

    /** Red left trench (higher Y, red side of field). */
	TRENCH_RED_LEFT(new Pose2d(11.9149, 6.6385, Rotation2d.fromDegrees(-90)), false, ElementType.TRENCH),

    // ----- OUTPOSTS -----

    /** Blue outpost — human player station on the blue alliance wall. */
    OUTPOST_BLUE(new Pose2d(0, 0.665988, Rotation2d.fromDegrees(0)), true, ElementType.OUTPOST),

    /** Red outpost — human player station on the red alliance wall. */
    OUTPOST_RED(new Pose2d(16.621, 7.403338, Rotation2d.fromDegrees(180)), false, ElementType.OUTPOST);

    private final Pose2d center;
    private final boolean isBlue;
    private final ElementType type;

    GameElement(Pose2d center, boolean isBlue, ElementType type) {
      this.center = center;
      this.isBlue = isBlue;
      this.type = type;
    }

    GameElement(Pose2d center, boolean isBlue) {
      this(center, isBlue, null);
    }

    public Pose2d getCenter() {
      return center;
    }

    public Translation2d getLocation() {
      return center.getTranslation();
    }

    public boolean isBlue() {
      return isBlue;
    }

    public ElementType getType() {
      return type;
    }

    public static List<GameElement> getColor(boolean isBlue) {
      return Arrays.stream(values())
          .filter(e -> e.isBlue() == isBlue)
          .collect(Collectors.toList());
    }

    public static List<GameElement> getByType(ElementType type) {
      return Arrays.stream(values())
          .filter(e -> e.type == type)
          .collect(Collectors.toList());
    }

    public static Pose2d getPoseWithOffset(GameElement element, double offsetMeters) {
      return offsetByAngle(element.center, offsetMeters, 0);
    }

    @Override
    public String toString() {
      return String.format("%s [Pose=%s, isBlue=%b, type=%s]", name(), center, isBlue, type);
    }
  }

  // ---- Utility Methods ----

  /** Normalizes an angle (radians) to [-pi, pi]. */
  public static double normalizeAngle(double angle) {
    while (angle > Math.PI) angle -= 2 * Math.PI;
    while (angle < -Math.PI) angle += 2 * Math.PI;
    return angle;
  }

  public static Pose2d getAllianceHub(){
    if (DriverStation.getAlliance().isPresent()) {
      if (DriverStation.getAlliance().get() == Alliance.Red) {
          return GameElement.HUB_RED.getCenter();
      }
    }
    return GameElement.HUB_BLUE.getCenter();
  }

	public static Pose2d getNearestTrench(Pose2d robotPose) {
    boolean isBlue = !DriverStation.getAlliance().isPresent() || DriverStation.getAlliance().get() != Alliance.Red;
    return GameElement.getByType(ElementType.TRENCH).stream()
      .filter(e -> e.isBlue() == isBlue)
      .min((t1, t2) -> Double.compare(
        robotPose.getTranslation().getDistance(t1.getLocation()),
        robotPose.getTranslation().getDistance(t2.getLocation())))
      .map(GameElement::getCenter)
        .orElse(null);
	}

  /**
   * Returns a Pose2d offset from a center pose by a distance at an angle. Keeps the original
   * rotation.
   */
  public static Pose2d offsetByAngle(
      Pose2d center, double offsetMeters, double angleOffsetDegrees) {
    Rotation2d offsetRotation =
        center.getRotation().plus(Rotation2d.fromDegrees(angleOffsetDegrees));
    double newX = center.getX() + offsetMeters * Math.cos(offsetRotation.getRadians());
    double newY = center.getY() + offsetMeters * Math.sin(offsetRotation.getRadians());
    return new Pose2d(newX, newY, center.getRotation());
  }

  /**
   * Returns the absolute angle difference (degrees) between a robot's heading and the direction
   * from the robot to an element.
   */
  public static double calculateAngleDifference(Pose2d robotPose, Pose2d elementPose) {
    Translation2d directionToElement =
        elementPose.getTranslation().minus(robotPose.getTranslation());
    Rotation2d elementDirection =
        new Rotation2d(directionToElement.getX(), directionToElement.getY());
    return Math.abs(robotPose.getRotation().minus(elementDirection).getDegrees());
  }
}
