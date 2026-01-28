package frc.robot.subsystems.drive;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightSubsystem;

/* TODO: Rough overview of what Odometry.java does and how it works.
 * Odometry tells the driver where the robot is at all times.
 * It's also VERY VERY useful for auton since during that period
 * the robot must know where it is to properly drive to each place
 * 
 * The robot can tell where it is via 2 ways: Limelights/Vision and Dead-Reckoning or using Motor Encoders
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
	private Pose2d robotPose;

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

    @Override
    public void periodic() {
		// Gets the interpolated value of each limelight (wont interpolate if only one april tag is present!)
        Pose2d visionPose = limelights.getBotPoseInterpolated();

		// If no april tags are present, do nothing pretty much
        if (!(visionPose.getX() == 0.0 && visionPose.getY() == 0.0 && visionPose.getRotation().getDegrees() == 0.0)) {
			// Gets the current timestamp
			double time = Utils.getCurrentTimeSeconds();

			// Adds the interpolated vision position to the Pigeon2 rotation into one Pose2d
            //Pose2d composite = new Pose2d(visionPose.getTranslation(), drive.getRotation3d().toRotation2d());

			// Adds values to SmartDashboard (or Elastic if u use that)
			// Shows where Odometry/Limelight thinks the robot is on the field
            SmartDashboard.putNumber("Odometry/X", robotPose.getX());
            SmartDashboard.putNumber("Odometry/Y", robotPose.getY());
            SmartDashboard.putNumber("Odometry/Rotation", robotPose.getRotation().getDegrees());

			// Add's two vision measure ments to SwerveDrive so that they can be filtered by it
			drive.addVisionMeasurement(limelights.getBotPoseFront(), time);
			drive.addVisionMeasurement(limelights.getBotPoseBack(), time);

			// Sets SwerveDrive's robot position to the composited robot position
            //drive.resetPose(composite);
        }

		// Sets the Odometry robotPose variable (for easy/more normal access to the robot position)
        robotPose = drive.getPose();

		// Puts a new field value into SmartDashboard (or Elastic if u use that)
        field.setRobotPose(robotPose);
        field.setRobotPose(robotPose.getX(), robotPose.getY(), robotPose.getRotation());
        SmartDashboard.putData(field);

		// Tells you in SmartDashboard (or Elastic if u use that) whether or not the code is interpolating apriltag positions
		SmartDashboard.putBoolean("Interpolating?", limelights.hasTarget());
    }
}
