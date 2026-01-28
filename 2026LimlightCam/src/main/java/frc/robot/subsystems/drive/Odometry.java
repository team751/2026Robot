package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.LimelightSubsystem;

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
        Pose2d visionPose = limelights.getBotPoseInterpolated();
        if (!(visionPose.getX() == 0.0 && visionPose.getY() == 0.0 && visionPose.getRotation().getDegrees() == 0.0)) {

            Pose2d composite = new Pose2d(visionPose.getTranslation(), drive.getRotation3d().toRotation2d());
            SmartDashboard.putNumber("Odometry/X", robotPose.getX());
            SmartDashboard.putNumber("Odometry/Y", robotPose.getY());
            SmartDashboard.putNumber("Odometry/Rotation", robotPose.getRotation().getDegrees());

            drive.resetPose(composite);
        }

        robotPose = drive.getPose();

        field.setRobotPose(robotPose);
        field.setRobotPose(robotPose.getX(), robotPose.getY(), robotPose.getRotation());
        SmartDashboard.putData(field);
		SmartDashboard.putBoolean("Interpolating?", limelights.hasTarget());
    }
}
