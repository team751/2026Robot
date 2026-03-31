package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.LimelightHelpers;

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

  public void resetPose(Pose2d newPose) {
    drive.resetPose(newPose);
  }

  private boolean isValidEstimate(LimelightHelpers.PoseEstimate estimate) {
    if (estimate == null) return false;
    if (estimate.tagCount == 0) return false;
    if (estimate.pose.getX() == 0.0 && estimate.pose.getY() == 0.0) return false;
    return true;
  }

  @Override
  public void periodic() {
    LimelightHelpers.PoseEstimate frontEstimate = limelights.getBotPoseFront(Constants.MEGATAG_2_USAGE);
    if (isValidEstimate(frontEstimate)) {
      drive.addVisionMeasurement(
          new Pose2d(frontEstimate.pose.getX(), frontEstimate.pose.getY(), drive.getPose().getRotation()),
          frontEstimate.timestampSeconds);
    }

    LimelightHelpers.PoseEstimate sideEstimate = limelights.getBotPoseSide(Constants.MEGATAG_2_USAGE);
    if (isValidEstimate(sideEstimate)) {
      drive.addVisionMeasurement(
          new Pose2d(sideEstimate.pose.getX(), sideEstimate.pose.getY(), drive.getPose().getRotation()),
          sideEstimate.timestampSeconds);
    }

    robotPose = drive.getPose();
    field.setRobotPose(robotPose.getX(), robotPose.getY(), robotPose.getRotation());
    SmartDashboard.putData(field);
  }
}
