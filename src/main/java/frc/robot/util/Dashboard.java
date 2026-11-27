package frc.robot.util;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Single place for all dashboard (Elastic) values.
 *
 * <p>WPILib 2027 removes SmartDashboard, SendableChooser and Sendable (replaced by the Telemetry
 * and Tunables APIs). Every former {@code SmartDashboard.putX(...)} call in the robot code now goes
 * through this class, so the 2027 change only has to be made here, not in every subsystem. This is
 * the one file allowed to call SmartDashboard directly ({@link #putField}).
 *
 * <p>TODO(2027): This is a framework stand-in and has NOT been compiled or tested against WPILib
 * 2027. For now it publishes straight to NetworkTables under the same "SmartDashboard/..." keys the
 * Elastic layout already uses. Once the exact Telemetry API (see the "Migration from WPILib 2026"
 * sections in allwpilib's telemetry.md and tunables.md) is confirmed, swap the bodies below to
 * Telemetry calls and keep the method signatures. The NT import paths will be changed to
 * org.wpilib.* by the 2027 project importer.
 *
 * <p>WARNING: the keys do NOT stay the same by default after the Telemetry swap. At alpha 7,
 * RobotBase registers the default backend as {@code new NetworkTablesTelemetryBackend(inst,
 * "/Telemetry")} for every path, so {@code Telemetry.log("Swerve/Pose x", ...)} lands at {@code
 * /Telemetry/Swerve/Pose x} and every Elastic tile goes blank. Pick one: (a) re-point the Elastic
 * layout to {@code /Telemetry/...}, or (b) keep the old keys by registering {@code
 * TelemetryRegistry.registerBackend("/SmartDashboard", new
 * NetworkTablesTelemetryBackend(NetworkTableInstance.getDefault(), ""))} in the Robot constructor
 * and logging to {@code "SmartDashboard/" + key} here. The longest-prefix match picks that backend
 * and it adds no prefix, so the NT key is {@code /SmartDashboard/<key>}. (b) was worked out from
 * the alpha-7 source (TelemetryRegistry, NetworkTablesTelemetryBackend, PathUtil) and has not been
 * run: check the real keys in OutlineViewer or AdvantageScope before relying on it.
 */
public final class Dashboard {
  private Dashboard() {}

  private static final NetworkTable table =
      NetworkTableInstance.getDefault().getTable("SmartDashboard");

  // Publishers are created once per key and reused; creating one every loop is slow.
  // Concurrent maps because the maple-sim code calls this from the 200Hz sim Notifier thread.
  private static final Map<String, DoublePublisher> doubles = new ConcurrentHashMap<>();
  private static final Map<String, BooleanPublisher> booleans = new ConcurrentHashMap<>();
  private static final Map<String, StringPublisher> strings = new ConcurrentHashMap<>();

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
   * Publishes a {@link Field2d} under {@code SmartDashboard/<key>}. Safe to call every loop:
   * putData does nothing if the same object is already stored under that key. Callers update the
   * pose with {@code field.setRobotPose(...)}.
   *
   * <p>Main robot loop only (SmartDashboard.putData is not called from the sim thread).
   *
   * <p>TODO(2027): Field2d still exists in 2027 (org.wpilib.smartdashboard.Field2d, now backed by
   * Telemetry/Tunables). Per telemetry.md "Complex Sendable Values", replace the body with {@code
   * Telemetry.log(key, field)}.
   */
  public static void putField(String key, Field2d field) {
    SmartDashboard.putData(key, field);
  }
}
