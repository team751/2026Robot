package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase {
  private static PhotonVisionSubsystem instance;

  public static PhotonVisionSubsystem getInstance() {
    if (instance == null) instance = new PhotonVisionSubsystem();
    return instance;
  }

  private final AprilTagFieldLayout fieldTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  private final PhotonCamera frontCamera =
      new PhotonCamera(PhotonVisionConstants.FRONT_CAMERA_NAME);
  private final PhotonPoseEstimator frontEstimator =
      new PhotonPoseEstimator(fieldTagLayout, PhotonVisionConstants.FRONT_CAMERA_OFFSET);

  private final PhotonCamera sideCamera = new PhotonCamera(PhotonVisionConstants.SIDE_CAMERA_NAME);
  private final PhotonPoseEstimator sideEstimator =
      new PhotonPoseEstimator(fieldTagLayout, PhotonVisionConstants.SIDE_CAMERA_OFFSET);

  // Poses of the tags each camera's estimator actually used, for AdvantageScope's 3D-field
  // "Vision Target" object type (draws a line from the robot to each pose).
  private final StructArrayPublisher<Pose3d> frontTagsPub =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Vision/Front/TagsUsed", Pose3d.struct)
          .publish();
  private final StructArrayPublisher<Pose3d> sideTagsPub =
      NetworkTableInstance.getDefault()
          .getStructArrayTopic("Vision/Side/TagsUsed", Pose3d.struct)
          .publish();

  private PhotonVisionSubsystem() {}

  // Package-private: only PhotonVisionSim (same package) needs the raw cameras to attach
  // simulated feeds. Odometry only ever needs the estimate lists below.
  PhotonCamera getFrontCamera() {
    return frontCamera;
  }

  PhotonCamera getSideCamera() {
    return sideCamera;
  }

  private List<EstimatedRobotPose> getEstimates(
      PhotonCamera camera, PhotonPoseEstimator estimator, StructArrayPublisher<Pose3d> tagsPub) {
    List<EstimatedRobotPose> estimates = new ArrayList<>();
    List<Pose3d> tagPoses = new ArrayList<>();

    List<PhotonPipelineResult> results = camera.getAllUnreadResults();
    for (PhotonPipelineResult result : results) {
      // The classic estimator.update(result) is @Deprecated(forRemoval) in photonlib 2026 —
      // multi-tag first, single-tag lowest-ambiguity as the fallback, done by hand instead.
      Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);
      if (estimate.isEmpty()) {
        estimate = estimator.estimateLowestAmbiguityPose(result);
      }
      estimate.ifPresent(estimates::add);

      estimate.ifPresent(
          e -> {
            for (PhotonTrackedTarget target : e.targetsUsed) {
              fieldTagLayout.getTagPose(target.getFiducialId()).ifPresent(tagPoses::add);
            }
          });
    }

    // Only republish when fresh frames actually arrived. The cameras run below the 50Hz robot
    // loop, so many loops legitimately drain zero frames — clearing the topic on those would make
    // the sightlines flicker. Fresh frames with no tags publishes an empty array, so lines
    // disappear promptly when tags genuinely leave view.
    if (!results.isEmpty()) {
      tagsPub.set(tagPoses.toArray(new Pose3d[0]));
    }

    return estimates;
  }

  public List<EstimatedRobotPose> getFrontEstimates() {
    return getEstimates(frontCamera, frontEstimator, frontTagsPub);
  }

  public List<EstimatedRobotPose> getSideEstimates() {
    return getEstimates(sideCamera, sideEstimator, sideTagsPub);
  }
}
