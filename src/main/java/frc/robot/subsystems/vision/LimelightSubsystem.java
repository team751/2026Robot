package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.List;
import java.util.Optional;
import limelight.Limelight;
import limelight.networktables.LimelightResults;
import limelight.networktables.LimelightSettings;

public class LimelightSubsystem extends SubsystemBase {

	private static LimelightSubsystem instance;

	private final Limelight limelight;

	public static LimelightSubsystem getInstance() {
		if (instance == null) instance = new LimelightSubsystem();
		return instance;
	}

	private LimelightSubsystem() {
		limelight = new Limelight(VisionConstants.Limelight.name);

		limelight
				.getSettings()
				.withCameraOffset(
						new Pose3d(
								0,
								VisionConstants.Limelight.yOffset.in(Units.Meters),
								VisionConstants.Limelight.zOffset.in(Units.Meters),
								new Rotation3d()));
	}

	public boolean hasTarget() {
		Optional<LimelightResults> results = limelight.getData().getResults();
		return results.map(limelightResults -> limelightResults.valid).orElse(false);
	}

	public void setLEDMode(LimelightSettings.LEDMode mode) {
		limelight.getSettings().withLimelightLEDMode(mode);
	}

	public Optional<Pose2d> getEstimatedPose() {
		return limelight.getData().getResults().map(LimelightResults::getBotPose2d);
	}

	@Override
	public void periodic() {}

	public void setAprilTagFilters() {
		Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
		List<Double> blueList = List.of();
		List<Double> redList = List.of();
		boolean isBlue = alliance.filter(value -> value == DriverStation.Alliance.Blue).isPresent();

		limelight.getSettings().withArilTagIdFilter(isBlue ? blueList : redList);
	}
}
