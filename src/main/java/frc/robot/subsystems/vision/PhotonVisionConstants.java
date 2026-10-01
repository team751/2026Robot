package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class PhotonVisionConstants {
  // Must match the camera nicknames in the PhotonVision web UI exactly, or the PhotonCamera
  // silently subscribes to an empty NetworkTables path.
  public static final String FRONT_CAMERA_NAME = "front";
  public static final String SIDE_CAMERA_NAME = "side";

  // Static IP of the coprocessor (WPILib convention: static devices live in 10.TE.AM.6-.19).
  public static final String COPROCESSOR_IP = "10.7.51.11";

  // Used by the single-tag fallback on the RIO. The coprocessor's multi-tag solve uses whatever
  // layout is selected in the PV web UI, so the two MUST match (welded vs AndyMark, per event).
  public static final AprilTagFieldLayout FIELD_LAYOUT =
      AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

  // --- Camera mounting (robot center at floor level -> camera lens) ---
  //
  // WPILib convention, NOT Limelight convention:
  //   X + forward, Y + LEFT, Z + up
  //   pitch + = nose DOWN (so a camera tilted up has negative pitch)
  //   yaw   + = turned LEFT (counter-clockwise from above)
  // Sim cannot catch a wrong sign here (it places and reads the camera with the same transform),
  // so these have to be checked on the real robot. See docs/vision-and-odometry.md.

  // TODO(measure): both offsets below are placeholders copied from the old Limelight mounts.
  // The pitch was converted to WPILib convention but the Y values were copied as-is, and the old
  // Limelight comments disagree about which way +Y pointed, so Y may be mirrored on BOTH cameras.
  // Re-measure X/Y/Z and angles on the PV mounts.
  public static final Transform3d FRONT_CAMERA_OFFSET =
      new Transform3d(
          new Translation3d(0.0416, -0.1453, 0.4128), // meters
          new Rotation3d(0, Units.degreesToRadians(-30), 0)); // tilted 30 deg UP

  // The side yaw sign is unverified too: +90 if the lens points out the robot's LEFT side, -90 if
  // it points out the RIGHT side. (As written, +Y with -90 means "left of center, facing right".)
  public static final Transform3d SIDE_CAMERA_OFFSET =
      new Transform3d(
          new Translation3d(-0.2539, 0.1119, 0.2906), // meters
          new Rotation3d(0, 0, Units.degreesToRadians(-90)));

  // --- Fusion trust (x, y, theta std devs; higher = trust vision less) ---
  // Starting values from PhotonVision's official pose estimation example. Theta is 99999 so
  // heading always comes from the Pigeon, never from vision.
  public static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4.0, 4.0, 99999.0);
  public static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 99999.0);
  // Std devs are scaled by (1 + distance^2 / this), so far tags are trusted much less than close
  // ones. Bigger number = distance matters less.
  public static final double STD_DEV_DISTANCE_DIVISOR = 30.0;
  // Two tags on the same face, seen from far away, are almost one flat target and can flip like
  // a single tag. Past this distance a 2-tag multi-tag solve only gets single-tag trust.
  public static final double FAR_TAG_PAIR_DIST_M = 5.0;

  // --- Rejection filters (an estimate failing any of these is never fused) ---
  // Single-tag only: PhotonVision's own docs say ambiguity above 0.2 is likely a flipped solve.
  public static final double MAX_AMBIGUITY = 0.2;
  // Single-tag only: past this, one tag's corners are too few pixels to trust.
  public static final double MAX_SINGLE_TAG_DIST_M = 4.0;
  // The robot is on the floor, so an estimate floating above/below it is a bad solve or a bad
  // camera pitch.
  public static final double MAX_Z_ERROR_M = 0.25;
  // Capture-to-now. Negative or huge means time sync with the coprocessor is broken.
  public static final double MAX_LATENCY_S = 0.5;
  // How far outside the field walls an estimate can land before it's thrown out.
  public static final double FIELD_BORDER_MARGIN_M = 0.5;
  // Don't fuse vision while spinning faster than this (deg/s).
  public static final double MAX_ANGULAR_VELOCITY_DPS = 720.0;

  // --- Pose stability (dashboard indicator only; nothing gates on it) ---
  // Vision must agree with odometry within this distance (meters) to count as stable
  public static final double POSE_STABLE_EPSILON_METERS = 0.10;
  // Number of consecutive agreeing updates before pose is considered stable
  public static final int POSE_STABLE_THRESHOLD = 10;
  // A camera with no accepted estimate for this long stops counting as stable
  public static final double POSE_STABLE_TIMEOUT_S = 0.5;
}
