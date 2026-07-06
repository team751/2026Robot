package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

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
      PhotonCamera camera, PhotonPoseEstimator estimator) {
    List<EstimatedRobotPose> estimates = new ArrayList<>();
    for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
      // The classic estimator.update(result) is @Deprecated(forRemoval) in photonlib 2026 —
      // multi-tag first, single-tag lowest-ambiguity as the fallback, done by hand instead.
      Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);
      if (estimate.isEmpty()) {
        estimate = estimator.estimateLowestAmbiguityPose(result);
      }
      estimate.ifPresent(estimates::add);
    }
    return estimates;
  }

  public List<EstimatedRobotPose> getFrontEstimates() {
    return getEstimates(frontCamera, frontEstimator);
  }

  public List<EstimatedRobotPose> getSideEstimates() {
    return getEstimates(sideCamera, sideEstimator);
  }
}
