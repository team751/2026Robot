package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class PhotonVisionSubsystem extends SubsystemBase {
  AprilTagFieldLayout fieldTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  PhotonCamera frontcamera = new PhotonCamera(PhotonVisionConstants.FRONT_CAMERA_NAME);
  PhotonPoseEstimator frontEstimator =
      new PhotonPoseEstimator(fieldTagLayout, PhotonVisionConstants.FRONT_CAMERA_OFFSET);

  PhotonCamera sidecamera = new PhotonCamera(PhotonVisionConstants.SIDE_CAMERA_NAME);
  PhotonPoseEstimator sideEstimator =
      new PhotonPoseEstimator(fieldTagLayout, PhotonVisionConstants.SIDE_CAMERA_OFFSET);
}