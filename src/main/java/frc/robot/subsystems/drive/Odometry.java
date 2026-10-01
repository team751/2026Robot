package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.PhotonVisionConstants;
import frc.robot.subsystems.vision.PhotonVisionSubsystem;
import frc.robot.subsystems.vision.PhotonVisionSubsystem.VisionObservation;
import frc.robot.util.FieldConstants;
import java.util.List;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

/* Rough overview of what Odometry.java does and how it works.
 * Odometry tells the driver where the robot is at all times.
 * It's also VERY VERY useful for auton since during that period
 * the robot must know where it is to properly drive to each place
 *
 * The robot can tell where it is using 2 ways: Vision (PhotonVision cameras) and Dead-Reckoning
 * (using Motor Encoders)
 *
 * Vision involves using cameras to read AprilTags (glorified QR codes) to determine
 * where the robot is. PhotonVision does a whole bunch of math (that we dont care about)
 * and then tells us where it thinks the robot is. Unfortunately, we cant always see
 * an AprilTag at all times, and sometimes a camera is confidently wrong.
 *
 * Dead-Reckoning, uses the Motor Encoders to determine where the robot is.
 * It measures every rotation of the wheels and the velocity to tell where it is
 * relative to its last known position. Compared to Vision, this is a lot less accurate
 * since the wheels can slip or we could get hit and move meaning our position is
 * off and no longer accurate.
 *
 * Luckily, we can add these two together to be very very accurate (more or less) as
 * to where the robot is at all times! Every vision estimate first goes through a set of
 * sanity checks (getRejectReason). The ones that pass get blended into the pose, weighted by how
 * much we trust them (computeStdDevs): many close tags = lots of trust, one far tag = very little.
 * When we no longer see an AprilTag, SwerveDrive keeps going with Dead-Reckoning until we
 * see another AprilTag!
 */

public class Odometry extends SubsystemBase {
  private static Odometry instance;
  private final SwerveSubsystem drive;
  private final PhotonVisionSubsystem photon;
  private Field2d field = new Field2d();
  public Pose2d robotPose;

  /** Per-camera bookkeeping, for the dashboard only. */
  private static class CameraStats {
    private final String prefix;
    private int stableCount = 0;
    private int acceptedCount = 0;
    private int rejectedCount = 0;
    private double lastAcceptedTime = Double.NEGATIVE_INFINITY;

    CameraStats(String prefix) {
      this.prefix = prefix;
    }

    boolean isStable() {
      return stableCount >= PhotonVisionConstants.POSE_STABLE_THRESHOLD;
    }
  }

  private final CameraStats front = new CameraStats("Vision/Front");
  private final CameraStats side = new CameraStats("Vision/Side");

  private Odometry() {
    this.drive = SwerveSubsystem.getInstance();
    this.photon = PhotonVisionSubsystem.getInstance();
    this.robotPose = new Pose2d();
  }

  public static Odometry getInstance() {
    if (instance == null) {
      // Created first on purpose: subsystems' periodic() run in the order they were created, so
      // this way PhotonVision reads this loop's camera frames before Odometry fuses them.
      PhotonVisionSubsystem.getInstance();
      instance = new Odometry();
    }
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
    return front.isStable();
  }

  public boolean isSideStable() {
    return side.isStable();
  }

  private boolean isRotatingTooFast() {
    double yawRate = Math.abs(drive.getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
    return yawRate > PhotonVisionConstants.MAX_ANGULAR_VELOCITY_DPS;
  }

  /**
   * @return why this estimate shouldn't be trusted, or null if it passes every check.
   */
  private String getRejectReason(VisionObservation observation) {
    Pose3d pose = observation.estimate().estimatedPose;
    double timestamp = observation.estimate().timestampSeconds;

    // Every check below is "reject if value > limit", and any comparison with NaN is false, so
    // a NaN would sail through all of them and then poison the pose for the rest of the match.
    if (!Double.isFinite(pose.getX())
        || !Double.isFinite(pose.getY())
        || !Double.isFinite(pose.getZ())
        || !Double.isFinite(timestamp)
        || !Double.isFinite(observation.avgTagDistanceMeters())) {
      return "non-finite";
    }

    double latency = Timer.getFPGATimestamp() - timestamp;
    if (latency < 0 || latency > PhotonVisionConstants.MAX_LATENCY_S) return "latency";

    // Seen before the last pose reset: it describes where the robot was before the reset.
    // (Checked after latency, so broken time sync shows up as "latency", not as this.)
    if (timestamp < drive.getLastResetFpgaTime()) return "before reset";

    if (Math.abs(pose.getZ()) > PhotonVisionConstants.MAX_Z_ERROR_M) return "not on floor";

    double margin = PhotonVisionConstants.FIELD_BORDER_MARGIN_M;
    if (pose.getX() < -margin
        || pose.getX() > FieldConstants.FIELD_LENGTH + margin
        || pose.getY() < -margin
        || pose.getY() > FieldConstants.FIELD_WIDTH + margin) {
      return "off field";
    }

    // A single tag can "flip" (two poses that look almost identical to the camera).
    if (observation.estimate().strategy == PoseStrategy.LOWEST_AMBIGUITY) {
      if (!Double.isFinite(observation.ambiguity())
          || observation.ambiguity() > PhotonVisionConstants.MAX_AMBIGUITY) {
        return "ambiguous";
      }
      if (observation.avgTagDistanceMeters() > PhotonVisionConstants.MAX_SINGLE_TAG_DIST_M) {
        return "single tag too far";
      }
    }

    return null;
  }

  /** Less trust (bigger std devs) for single tags and for far-away tags. */
  private Matrix<N3, N1> computeStdDevs(VisionObservation observation) {
    double distance = observation.avgTagDistanceMeters();

    // Decided by strategy, not tag count alone: the single-tag fallback can run on a frame that
    // has several tags in view (e.g. multi-tag switched off in the PV UI). And a far-away pair of
    // tags is nearly one flat target, so it only gets single-tag trust.
    boolean isMultiTag =
        observation.estimate().strategy == PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    boolean isFarTagPair =
        observation.tagCount() <= 2 && distance > PhotonVisionConstants.FAR_TAG_PAIR_DIST_M;
    Matrix<N3, N1> base =
        (isMultiTag && !isFarTagPair)
            ? PhotonVisionConstants.MULTI_TAG_STD_DEVS
            : PhotonVisionConstants.SINGLE_TAG_STD_DEVS;

    return base.times(1 + (distance * distance) / PhotonVisionConstants.STD_DEV_DISTANCE_DIVISOR);
  }

  /** Filters and fuses one camera's estimates from this loop. */
  private void fuseCamera(List<VisionObservation> observations, CameraStats stats) {
    for (VisionObservation observation : observations) {
      String rejectReason = getRejectReason(observation);
      if (rejectReason != null) {
        // Only written on rejection, so the last reason stays on the dashboard instead of being
        // instantly overwritten by the next good frame.
        SmartDashboard.putString(stats.prefix + "/RejectReason", rejectReason);
        stats.rejectedCount++;
        continue;
      }

      // Heading always comes from the gyro (theta std dev is 99999 anyway).
      Pose2d visionPose =
          new Pose2d(
              observation.estimate().estimatedPose.getX(),
              observation.estimate().estimatedPose.getY(),
              drive.getPose().getRotation());

      // estimate.timestampSeconds is FPGA time (PhotonVision time-syncs with the RIO), but CTRE's
      // addVisionMeasurement wants the Utils.getCurrentTimeSeconds() timebase. fpgaToCurrentTime
      // converts between the two clocks.
      double timestamp = Utils.fpgaToCurrentTime(observation.estimate().timestampSeconds);

      // Stability: does vision agree with where odometry thought the robot was when the frame
      // was CAPTURED? (Not now: at full speed the robot moves ~0.2 m during camera latency.)
      // Sampled before fusing so this frame's own correction can't make it agree with itself.
      Pose2d poseAtCapture = drive.samplePoseAt(timestamp).orElse(drive.getPose());
      double error = poseAtCapture.getTranslation().getDistance(visionPose.getTranslation());
      stats.stableCount =
          (error < PhotonVisionConstants.POSE_STABLE_EPSILON_METERS) ? stats.stableCount + 1 : 0;

      drive.addVisionMeasurement(visionPose, timestamp, computeStdDevs(observation));
      stats.acceptedCount++;
      stats.lastAcceptedTime = Timer.getFPGATimestamp();
    }
  }

  private void publishStats(CameraStats stats) {
    // No vision for a while (lost the tags, or nothing passing the filters): stop claiming stable.
    if (Timer.getFPGATimestamp() - stats.lastAcceptedTime
        > PhotonVisionConstants.POSE_STABLE_TIMEOUT_S) {
      stats.stableCount = 0;
    }
    SmartDashboard.putNumber(stats.prefix + "/AcceptedCount", stats.acceptedCount);
    SmartDashboard.putNumber(stats.prefix + "/RejectedCount", stats.rejectedCount);
  }

  @Override
  public void periodic() {
    // PhotonVisionSubsystem.periodic() already drained this loop's frames, so reading them here
    // is free. Frames captured mid-spin are simply dropped instead of piling up for later.
    if (isRotatingTooFast()) {
      SmartDashboard.putBoolean("Odometry/VisionRejected", true);
    } else {
      SmartDashboard.putBoolean("Odometry/VisionRejected", false);
      fuseCamera(photon.getFrontObservations(), front);
      fuseCamera(photon.getSideObservations(), side);
    }
    publishStats(front);
    publishStats(side);

    robotPose = drive.getPose();
    field.setRobotPose(robotPose.getX(), robotPose.getY(), robotPose.getRotation());

    SmartDashboard.putBoolean("Odometry/PoseStable", isPoseStable());
    SmartDashboard.putBoolean("Odometry/FrontStable", isFrontStable());
    SmartDashboard.putBoolean("Odometry/SideStable", isSideStable());
    SmartDashboard.putData(field);
  }
}
