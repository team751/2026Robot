package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
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

  /**
   * Call when the sim robot is teleported (SwerveSubsystem.resetPose). The fake cameras render each
   * frame from a pose interpolated over the last ~1.5 s, so without this they'd draw frames from
   * halfway along the "path" of the jump. Does nothing if vision sim isn't running.
   */
  public static void onTeleport(Pose2d pose) {
    if (instance != null) instance.visionSim.resetRobotPose(pose);
  }

  private final VisionSystemSim visionSim = new VisionSystemSim("photonvision");

  private PhotonVisionSim() {
    visionSim.addAprilTags(PhotonVisionConstants.FIELD_LAYOUT);

    PhotonVisionSubsystem photon = PhotonVisionSubsystem.getInstance();

    // PI4_LIFECAM presets are official PhotonVision sim profiles — placeholder until the real
    // camera hardware is chosen. FIELD_LAYOUT is passed explicitly: otherwise the simulated
    // multi-tag solve quietly uses WPILib's default field instead of ours.
    PhotonCameraSim frontCameraSim =
        new PhotonCameraSim(
            photon.getFrontCamera(),
            SimCameraProperties.PI4_LIFECAM_640_480(),
            PhotonVisionConstants.FIELD_LAYOUT);
    visionSim.addCamera(frontCameraSim, PhotonVisionConstants.FRONT_CAMERA_OFFSET);

    PhotonCameraSim sideCameraSim =
        new PhotonCameraSim(
            photon.getSideCamera(),
            SimCameraProperties.PI4_LIFECAM_640_480(),
            PhotonVisionConstants.FIELD_LAYOUT);
    visionSim.addCamera(sideCameraSim, PhotonVisionConstants.SIDE_CAMERA_OFFSET);
  }

  @Override
  public void periodic() {
    // The fake cameras look from where the robot REALLY is (MapleSim ground truth), not from
    // where odometry thinks it is — otherwise vision could never correct odometry drift.
    visionSim.update(SwerveSubsystem.getInstance().getGroundTruthPose());
  }
}
