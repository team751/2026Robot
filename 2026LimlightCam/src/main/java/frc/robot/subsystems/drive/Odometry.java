package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.LimelightConstants;
//import java.util.Optional;


public class Odometry extends SubsystemBase {
	private static Odometry instance;
	private final SwerveSubsystem drive;
	private final LimelightSubsystem limelight;
    private Field2d field = new Field2d();
	private Pose2d robotPose;

	public Odometry() {
		this.drive = SwerveSubsystem.getInstance();
		this.limelight = LimelightSubsystem.getInstance();
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

	private double lastTime = 0.0;

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();

        // Add vision measurement FIRST
        
        Pose2d visionPose = limelight.getBotPose();
        if (!(visionPose.getX() == 0.0 && visionPose.getY() == 0.0 && visionPose.getRotation().getDegrees() == 0.0)) {

            Pose2d composite = new Pose2d(visionPose.getTranslation(), drive.getRotation3d().toRotation2d());
            SmartDashboard.putNumber("Odometry/X", robotPose.getX());
            SmartDashboard.putNumber("Odometry/Y", robotPose.getY());
            SmartDashboard.putNumber("Odometry/Rotation", robotPose.getRotation().getDegrees());

            drive.resetPose(composite);
        }


        // THEN get the updated pose (after vision has been fused)
        robotPose = drive.getPose();


        field.setRobotPose(robotPose);
        field.setRobotPose(robotPose.getX(), robotPose.getY(), robotPose.getRotation());
        SmartDashboard.putData(field);

        // Publish diagnostics
    
        if (now - lastTime > 0.5) {
            lastTime = now;
            //System.out.println("Odometry: x=" + robotPose.getX() + " y=" + robotPose.getY() + " | rotation=" + robotPose.getRotation());
            //System.out.println(maybeVision.get());
        }
    }
}
