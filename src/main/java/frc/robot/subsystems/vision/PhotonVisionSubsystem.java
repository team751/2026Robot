package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/* Rough overview of PhotonVision
 * Two cameras plug into a coprocessor running PhotonVision, which finds AprilTags and publishes
 * what it sees over NetworkTables. This subsystem reads those frames once per loop, turns each
 * one into a robot pose estimate, and hands the list to Odometry.java, which decides whether
 * to trust each estimate and how much.
 */
public class PhotonVisionSubsystem extends SubsystemBase {
  private static PhotonVisionSubsystem instance;

  public static PhotonVisionSubsystem getInstance() {
    if (instance == null) instance = new PhotonVisionSubsystem();
    return instance;
  }

  /**
   * One pose estimate plus the facts Odometry needs to filter and weight it. photonlib's
   * EstimatedRobotPose.targetsUsed lists EVERY target in the frame, not just the ones the solve
   * used, so the real tags, distance, and quality numbers are worked out here instead.
   *
   * @param tagIds the tags the solve actually used
   * @param ambiguity single-tag flip risk, 0 to 1 (always 0 for multi-tag: one solution)
   * @param reprojErrPx multi-tag solve error in pixels (NaN for single-tag). High = bad calibration
   */
  public record VisionObservation(
      EstimatedRobotPose estimate,
      List<Integer> tagIds,
      double avgTagDistanceMeters,
      double ambiguity,
      double reprojErrPx) {
    public int tagCount() {
      return tagIds.size();
    }
  }

  private final Camera front =
      new Camera(
          PhotonVisionConstants.FRONT_CAMERA_NAME,
          PhotonVisionConstants.FRONT_CAMERA_OFFSET,
          "Vision/Front");
  private final Camera side =
      new Camera(
          PhotonVisionConstants.SIDE_CAMERA_NAME,
          PhotonVisionConstants.SIDE_CAMERA_OFFSET,
          "Vision/Side");

  private PhotonVisionSubsystem() {}

  // Package-private: only PhotonVisionSim (same package) needs the raw cameras to attach
  // simulated feeds. Odometry only ever needs the observation lists below.
  PhotonCamera getFrontCamera() {
    return front.camera;
  }

  PhotonCamera getSideCamera() {
    return side.camera;
  }

  /** Observations from frames that arrived since the last loop. Safe to call more than once. */
  public List<VisionObservation> getFrontObservations() {
    return front.observations;
  }

  public List<VisionObservation> getSideObservations() {
    return side.observations;
  }

  @Override
  public void periodic() {
    // getAllUnreadResults() drains a queue, so it must be called exactly once per loop. Doing it
    // here (instead of from Odometry) keeps that true no matter what Odometry decides to fuse.
    // Odometry.getInstance() creates this subsystem first, so this runs before Odometry each loop
    // (if it didn't, Odometry would just fuse each batch one loop later).
    front.update();
    side.update();
  }

  /** Everything for one physical camera: the camera, its estimator, and its telemetry. */
  private static class Camera {
    // Don't flood the Driver Station if a camera keeps failing every loop.
    private static final double ERROR_REPORT_INTERVAL_S = 5.0;

    private final PhotonCamera camera;
    private final PhotonPoseEstimator estimator;
    private final String telemetryPrefix;

    // Poses of the tags each estimate used, for AdvantageScope's 3D-field "Vision Target"
    // object type (draws a line from the robot to each pose).
    private final StructArrayPublisher<Pose3d> tagsPub;
    // What the camera thinks the robot pose is, before Odometry filters anything. Pose3d keeps
    // Z/roll/pitch, which is where a wrong camera offset shows up first.
    private final StructArrayPublisher<Pose2d> rawPosePub;
    private final StructArrayPublisher<Pose3d> rawPose3dPub;

    private List<VisionObservation> observations = List.of();
    private double lastErrorReportTime = Double.NEGATIVE_INFINITY;
    private boolean telemetryCleared = false;

    Camera(String name, Transform3d robotToCamera, String telemetryPrefix) {
      this.camera = new PhotonCamera(name);
      this.estimator = new PhotonPoseEstimator(PhotonVisionConstants.FIELD_LAYOUT, robotToCamera);
      this.telemetryPrefix = telemetryPrefix;

      NetworkTableInstance nt = NetworkTableInstance.getDefault();
      tagsPub = nt.getStructArrayTopic(telemetryPrefix + "/TagsUsed", Pose3d.struct).publish();
      rawPosePub = nt.getStructArrayTopic(telemetryPrefix + "/RawPose", Pose2d.struct).publish();
      rawPose3dPub =
          nt.getStructArrayTopic(telemetryPrefix + "/RawPose3d", Pose3d.struct).publish();
    }

    void update() {
      boolean connected = camera.isConnected();
      SmartDashboard.putBoolean(telemetryPrefix + "/Connected", connected);

      // photonlib throws if the coprocessor's PhotonVision message format doesn't match the
      // vendordep (i.e. a different PV version). Uncaught, that would abort the whole
      // CommandScheduler loop (skipping every command that loop) every 5 seconds. Instead, this
      // camera just reports no estimates.
      boolean failed = false;
      try {
        observations = readObservations();
      } catch (Exception e) {
        failed = true;
        observations = List.of();
        double now = Timer.getFPGATimestamp();
        if (now - lastErrorReportTime > ERROR_REPORT_INTERVAL_S) {
          DriverStation.reportError(
              "PhotonVision camera '" + camera.getName() + "' failed: " + e, e.getStackTrace());
          lastErrorReportTime = now;
        }
      }

      // A dead camera sends no frames, so nothing would ever replace its last pose on the
      // dashboard. Clear it so AdvantageScope doesn't keep drawing a pose nobody is seeing.
      if (!connected || failed) {
        publish(List.of());
      }
    }

    private List<VisionObservation> readObservations() {
      List<VisionObservation> results = new ArrayList<>();

      List<PhotonPipelineResult> frames = camera.getAllUnreadResults();
      for (PhotonPipelineResult frame : frames) {
        estimate(frame).ifPresent(results::add);
      }

      // Only republish when fresh frames actually arrived. The cameras run below the 50Hz robot
      // loop, so many loops legitimately drain zero frames — clearing the topics on those would
      // make AdvantageScope flicker. Fresh frames with no tags publish empty arrays, so poses
      // disappear promptly when tags genuinely leave view.
      if (!frames.isEmpty()) {
        publish(results);
      }

      return results;
    }

    private Optional<VisionObservation> estimate(PhotonPipelineResult frame) {
      // Multi-tag first: solved on the coprocessor from every visible tag at once, so it's far
      // harder to "flip" than a single tag. Empty unless multi-tag is enabled in the PV pipeline.
      Optional<EstimatedRobotPose> multiTag = estimator.estimateCoprocMultiTagPose(frame);
      if (multiTag.isPresent()) {
        MultiTargetPNPResult multiTagResult = frame.getMultiTagResult().get();
        List<Integer> tagIds =
            multiTagResult.fiducialIDsUsed.stream().map(Short::intValue).toList();
        return Optional.of(
            new VisionObservation(
                multiTag.get(),
                tagIds,
                averageDistance(frame.getTargets(), tagIds),
                multiTagResult.estimatedPose.ambiguity,
                multiTagResult.estimatedPose.bestReprojErr));
      }

      // Single-tag fallback. The classic estimator.update(result) is @Deprecated(forRemoval) in
      // photonlib 2026, so the fallback is done by hand.
      Optional<EstimatedRobotPose> singleTag = estimator.estimateLowestAmbiguityPose(frame);
      if (singleTag.isEmpty()) return Optional.empty();

      PhotonTrackedTarget used = lowestAmbiguityTarget(frame.getTargets());
      return Optional.of(
          new VisionObservation(
              singleTag.get(),
              List.of(used.getFiducialId()),
              distanceMeters(used),
              used.getPoseAmbiguity(),
              Double.NaN));
    }

    /**
     * The same target estimateLowestAmbiguityPose() picks. photonlib doesn't say which one it used,
     * so this copies its exact rule: lowest ambiguity below 10, skipping -1 (non-3D targets). Never
     * null when estimateLowestAmbiguityPose() returned a pose.
     */
    private static PhotonTrackedTarget lowestAmbiguityTarget(List<PhotonTrackedTarget> targets) {
      PhotonTrackedTarget best = null;
      double lowestAmbiguity = 10;
      for (PhotonTrackedTarget target : targets) {
        double ambiguity = target.getPoseAmbiguity();
        if (ambiguity != -1 && ambiguity < lowestAmbiguity) {
          lowestAmbiguity = ambiguity;
          best = target;
        }
      }
      return best;
    }

    /** Infinity if none of the IDs are in the frame, so Odometry rejects it as unknown. */
    private static double averageDistance(List<PhotonTrackedTarget> targets, List<Integer> ids) {
      double sum = 0.0;
      int count = 0;
      for (PhotonTrackedTarget target : targets) {
        if (ids.contains(target.getFiducialId())) {
          sum += distanceMeters(target);
          count++;
        }
      }
      return count == 0 ? Double.POSITIVE_INFINITY : sum / count;
    }

    private static double distanceMeters(PhotonTrackedTarget target) {
      return target.getBestCameraToTarget().getTranslation().getNorm();
    }

    private void publish(List<VisionObservation> results) {
      // Clearing writes NaN, and NaN never equals NaN, so NetworkTables can't skip repeats of it:
      // clearing every idle loop would write ~400 new log records/sec. So only clear once.
      if (results.isEmpty() && telemetryCleared) return;
      telemetryCleared = results.isEmpty();

      List<Pose3d> tagPoses = new ArrayList<>();
      for (VisionObservation observation : results) {
        for (int id : observation.tagIds()) {
          PhotonVisionConstants.FIELD_LAYOUT.getTagPose(id).ifPresent(tagPoses::add);
        }
      }
      tagsPub.set(tagPoses.toArray(new Pose3d[0]));
      rawPosePub.set(
          results.stream().map(o -> o.estimate().estimatedPose.toPose2d()).toArray(Pose2d[]::new));
      rawPose3dPub.set(
          results.stream().map(o -> o.estimate().estimatedPose).toArray(Pose3d[]::new));

      if (results.isEmpty()) {
        SmartDashboard.putString(telemetryPrefix + "/Strategy", "NONE");
        SmartDashboard.putNumber(telemetryPrefix + "/TagCount", 0);
        SmartDashboard.putNumber(telemetryPrefix + "/AvgTagDist", Double.NaN);
        SmartDashboard.putNumber(telemetryPrefix + "/Ambiguity", Double.NaN);
        SmartDashboard.putNumber(telemetryPrefix + "/ReprojErrPx", Double.NaN);
        SmartDashboard.putNumber(telemetryPrefix + "/LatencyMs", Double.NaN);
        return;
      }

      // Scalars describe the newest estimate. Strategy is the quickest way to spot a pipeline
      // with multi-tag accidentally switched off: it'll say LOWEST_AMBIGUITY with 2+ tags in view.
      VisionObservation latest = results.get(results.size() - 1);
      SmartDashboard.putString(telemetryPrefix + "/Strategy", latest.estimate().strategy.name());
      SmartDashboard.putNumber(telemetryPrefix + "/TagCount", latest.tagCount());
      SmartDashboard.putNumber(telemetryPrefix + "/AvgTagDist", latest.avgTagDistanceMeters());
      SmartDashboard.putNumber(telemetryPrefix + "/Ambiguity", latest.ambiguity());
      SmartDashboard.putNumber(telemetryPrefix + "/ReprojErrPx", latest.reprojErrPx());
      SmartDashboard.putNumber(
          telemetryPrefix + "/LatencyMs",
          (Timer.getFPGATimestamp() - latest.estimate().timestampSeconds) * 1000.0);
    }
  }
}
