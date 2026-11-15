package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import java.util.HashMap;
import java.util.Map;

/**
 * Single place for all dashboard (Elastic) values.
 *
 * <p>WPILib 2027 removes SmartDashboard, SendableChooser and Sendable (replaced by the Telemetry
 * and Tunables APIs). Every former {@code SmartDashboard.putX(...)} call in the robot code now goes
 * through this class, so the 2027 change only has to be made here, not in every subsystem.
 *
 * <p>TODO(2027): This is a framework stand-in and has NOT been compiled or tested against WPILib
 * 2027. It publishes straight to NetworkTables under the same "SmartDashboard/..." keys the Elastic
 * layout already uses, so the tile names do not change. Once the exact Telemetry API (see the
 * "Migration from WPILib 2026" sections in allwpilib's telemetry.md and tunables.md) is confirmed,
 * swap the bodies below to Telemetry calls and keep the method signatures. The NT import paths will
 * be changed to org.wpilib.* by the 2027 project importer.
 */
public final class Dashboard {
  private Dashboard() {}

  private static final NetworkTable table =
      NetworkTableInstance.getDefault().getTable("SmartDashboard");

  // Publishers are created once per key and reused; creating one every loop is slow.
  private static final Map<String, DoublePublisher> doubles = new HashMap<>();
  private static final Map<String, BooleanPublisher> booleans = new HashMap<>();
  private static final Map<String, StringPublisher> strings = new HashMap<>();
  private static final Map<String, DoubleArrayPublisher> fieldPoses = new HashMap<>();

  public static void putNumber(String key, double value) {
    doubles.computeIfAbsent(key, k -> table.getDoubleTopic(k).publish()).set(value);
  }

  public static void putBoolean(String key, boolean value) {
    booleans.computeIfAbsent(key, k -> table.getBooleanTopic(k).publish()).set(value);
  }

  public static void putString(String key, String value) {
    strings.computeIfAbsent(key, k -> table.getStringTopic(k).publish()).set(value);
  }

  /**
   * Replacement for {@code SmartDashboard.putData(Field2d)} plus {@code field.setRobotPose(...)}.
   * Publishes the same NetworkTables layout a Field2d widget reads (".type" = "Field2d" and a
   * "Robot" array of x, y, degrees), the same manual approach MapSimSwerveTelemetry already uses.
   *
   * <p>TODO(2027): replace with the Telemetry field/pose API once confirmed.
   */
  public static void putFieldRobotPose(String fieldName, Pose2d pose) {
    NetworkTable fieldTable = table.getSubTable(fieldName);
    fieldPoses
        .computeIfAbsent(
            fieldName,
            k -> {
              fieldTable.getStringTopic(".type").publish().set("Field2d");
              return fieldTable.getDoubleArrayTopic("Robot").publish();
            })
        .set(new double[] {pose.getX(), pose.getY(), pose.getRotation().getDegrees()});
  }
}
