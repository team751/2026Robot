package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveSubsystem;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/*
 * Sim-only: feeds the MapleSim ground-truth pose into a fake vision system so
 * PhotonCamera/PhotonPoseEstimator see NT data shaped exactly like a real coprocessor would
 * publish. Never constructed on the real robot (see Robot.robotInit()'s isSimulation() guard).
 */
public class PhotonVisionSim extends SubsystemBase {
  private static PhotonVisionSim instance;

  public static PhotonVisionSim getInstance() {
    if (instance == null) instance = new PhotonVisionSim();
    return instance;
  }

  private final VisionSystemSim visionSim = new VisionSystemSim("photonvision");

  private PhotonVisionSim() {
    AprilTagFieldLayout fieldLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    visionSim.addAprilTags(fieldLayout);

    PhotonVisionSubsystem photon = PhotonVisionSubsystem.getInstance();

    // PI4_LIFECAM presets are official PhotonVision sim profiles — placeholder until the real
    // camera hardware is chosen.
    PhotonCameraSim frontCameraSim =
        new PhotonCameraSim(photon.getFrontCamera(), SimCameraProperties.PI4_LIFECAM_640_480());
    visionSim.addCamera(frontCameraSim, PhotonVisionConstants.FRONT_CAMERA_OFFSET);

    PhotonCameraSim sideCameraSim =
        new PhotonCameraSim(photon.getSideCamera(), SimCameraProperties.PI4_LIFECAM_640_480());
    visionSim.addCamera(sideCameraSim, PhotonVisionConstants.SIDE_CAMERA_OFFSET);
  }

  @Override
  public void periodic() {
    // In sim, SwerveSubsystem.getPose() returns MapleSim's ground-truth pose — exactly what the
    // fake cameras should be looking from.
    visionSim.update(SwerveSubsystem.getInstance().getPose());
  }
}
