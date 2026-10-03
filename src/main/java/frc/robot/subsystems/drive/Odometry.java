package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightConstants;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.util.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

/* Rough overview of what Odometry.java does and how it works.
 * Odometry tells the driver where the robot is at all times.
 * It's also VERY VERY useful for auton since during that period
 * the robot must know where it is to properly drive to each place
 *
 * The robot can tell where it is using 2 ways: Limelights/Vision and Dead-Reckoning (using Motor Encoders)
 *
 * Vision involves using Limelights to read AprilTags (glorified QR codes) to determine
 * where the robot is. It does a whole bunch of math internally (that we dont care about)
 * and then tells us where it thinks the robot is. Unfortunately, we cant always see
 * an AprilTag at all times.
 *
 * Dead-Reckoning, uses the Motor Encoders to determine where the robot is.
 * It measures every rotation of the wheels and the velocity to tell where it is
 * relative to its last known position. Compared to Vision, this is a lot less accurate
 * since the wheels can slip or we could get hit and move meaning our position is
 * off and no longer accurate.
 *
 * Luckily, we can add these two together to be very very accurate (more or less) as
 * to where the robot is at all times! Whenever we see an AprilTag, we set our
 * robot postition there. When we no longer see an AprilTag, our last known position is
 * saved and SwerveDrive takes over with Dead-Reckoning to determine where we are until
 * we see another AprilTag!
 */

public class Odometry extends SubsystemBase {
  private static Odometry instance;
  private final SwerveSubsystem drive;
  private final LimelightSubsystem limelights;
  private Field2d field = new Field2d();
  public Pose2d robotPose;

  // Per-camera stability counters
  private int frontStableCount = 0;
  private int sideStableCount = 0;

  public Odometry() {
    this.drive = SwerveSubsystem.getInstance();
    this.limelights = LimelightSubsystem.getInstance();
    this.robotPose = new Pose2d();
  }

  public static Odometry getInstance() {
    if (instance == null) instance = new Odometry();
    return instance;
  }

  public Pose2d getPose() {
    return robotPose;
  }

  public Rotation2d getYaw() {
    return drive.getRotation();
  }

  public void resetPose(Pose2d newPose) {
    drive.resetPose(newPose);
  }

  public boolean isPoseStable() {
    return isFrontStable() && isSideStable();
  }

  public boolean isFrontStable() {
    return frontStableCount >= LimelightConstants.POSE_STABLE_THRESHOLD;
  }

  public boolean isSideStable() {
    return sideStableCount >= LimelightConstants.POSE_STABLE_THRESHOLD;
  }

  private boolean isValidEstimate(LimelightHelpers.PoseEstimate estimate) {
    if (estimate == null) return false;
    if (estimate.tagCount == 0) return false;
    if (estimate.pose.getX() == 0.0 && estimate.pose.getY() == 0.0) return false;
    return true;
  }

  private boolean isRotatingTooFast() {
    double yawRate = Math.abs(drive.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
    return yawRate > LimelightConstants.MAX_ANGULAR_VELOCITY_DPS;
  }

  private Matrix<N3, N1> computeStdDevs(
      Matrix<N3, N1> baseStdDevs, LimelightHelpers.PoseEstimate estimate) {
    Matrix<N3, N1> scaled = baseStdDevs.times(estimate.avgTagDist);
    if (estimate.tagCount > 1) {
      scaled = scaled.times(LimelightConstants.MULTI_TAG_STD_DEV_FACTOR);
    }
    return scaled;
  }

  /**
   * Logs the raw Limelight pose, or an empty array when no tags are seen. Logging an empty array
   * (rather than skipping the log entirely) means AdvantageScope's displayed pose actually clears
   * once tags are lost, instead of showing the last pose seen indefinitely.
   */
  private void logRawVisionPose(String telemetryPrefix, LimelightHelpers.PoseEstimate estimate) {
    if (isValidEstimate(estimate)) {
      Logger.recordOutput(telemetryPrefix + "/RawPose", new Pose2d[] {estimate.pose});
      Logger.recordOutput(telemetryPrefix + "/TagCount", estimate.tagCount);
      Logger.recordOutput(telemetryPrefix + "/AvgTagDist", estimate.avgTagDist);
      Logger.recordOutput(telemetryPrefix + "/Latency", estimate.latency);

      // Also on SmartDashboard for the Elastic layout
      SmartDashboard.putNumber(telemetryPrefix + "/RawX", estimate.pose.getX());
      SmartDashboard.putNumber(telemetryPrefix + "/RawY", estimate.pose.getY());
      SmartDashboard.putNumber(telemetryPrefix + "/TagCount", estimate.tagCount);
      SmartDashboard.putNumber(telemetryPrefix + "/AvgTagDist", estimate.avgTagDist);
    } else {
      Logger.recordOutput(telemetryPrefix + "/RawPose", new Pose2d[] {});
    }
  }

  /**
   * @return the stability count delta: +1 if vision agrees with odometry, reset to 0 if not, or -1
   *     if the estimate was rejected (caller should not update counter).
   */
  private int applyVisionEstimate(
      LimelightHelpers.PoseEstimate estimate, Matrix<N3, N1> baseStdDevs, String telemetryPrefix) {

    if (!isValidEstimate(estimate)) return -1;

    Pose2d visionPose =
        new Pose2d(estimate.pose.getX(), estimate.pose.getY(), drive.getPose().getRotation());

    // Use CTRE clock (same domain as odometry buffer) minus pipeline latency.
    // estimate.timestampSeconds is FPGA time, but CTRE uses JVM nanoTime internally —
    // different clocks cause measurements to fall outside the odometry buffer.
    double visionTimestamp = Utils.getCurrentTimeSeconds() - (estimate.latency / 1000.0);

    drive.addVisionMeasurement(visionPose, visionTimestamp, computeStdDevs(baseStdDevs, estimate));

    // Stability tracking
    double distance = drive.getPose().getTranslation().getDistance(estimate.pose.getTranslation());
    return (distance < LimelightConstants.POSE_STABLE_EPSILON_METERS) ? 1 : 0;
  }

  @Override
  public void periodic() {
    // Read each camera once per cycle and reuse the result, rather than querying NT twice
    // (once for logging, once for fusion) and risking the two reads disagreeing.
    LimelightHelpers.PoseEstimate frontEstimate = limelights.getBotPoseFront();
    LimelightHelpers.PoseEstimate sideEstimate = limelights.getBotPoseSide();

    // Log what the cameras see before any rejection, so rejected poses show up too
    logRawVisionPose("Vision/Front", frontEstimate);
    logRawVisionPose("Vision/Side", sideEstimate);

    boolean rejected = isRotatingTooFast();
    Logger.recordOutput("Odometry/VisionRejected", rejected);
    SmartDashboard.putBoolean("Odometry/VisionRejected", rejected);

    if (!rejected) {
      int frontResult =
          applyVisionEstimate(frontEstimate, LimelightConstants.FRONT_STD_DEVS, "Vision/Front");
      if (frontResult == 1) frontStableCount++;
      else if (frontResult == 0) frontStableCount = 0;

      int sideResult =
          applyVisionEstimate(sideEstimate, LimelightConstants.SIDE_STD_DEVS, "Vision/Side");
      if (sideResult == 1) sideStableCount++;
      else if (sideResult == 0) sideStableCount = 0;
    }

    robotPose = drive.getPose();

    Logger.recordOutput("Odometry/Robot", robotPose);
    Logger.recordOutput("Odometry/PoseStable", isPoseStable());
    Logger.recordOutput("Odometry/FrontStable", isFrontStable());
    Logger.recordOutput("Odometry/SideStable", isSideStable());
    Logger.recordOutput("Odometry/FrontStableCount", frontStableCount);
    Logger.recordOutput("Odometry/SideStableCount", sideStableCount);

    // Also on SmartDashboard for the Elastic layout
    field.setRobotPose(robotPose);
    SmartDashboard.putBoolean("Odometry/PoseStable", isPoseStable());
    SmartDashboard.putBoolean("Odometry/FrontStable", isFrontStable());
    SmartDashboard.putData(field);
  }
}
