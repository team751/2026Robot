package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.lights.LightsSubsystem;

public class Superstructure extends SubsystemBase {
	private static Superstructure instance = null;

	enum SuperstructureState {
		PRE_HOME,
		IDLE,
	}

	/* Subsystems */
	private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();
	private final ClimbSubsystem climbSubsystem = ClimbSubsystem.getInstance();
	private final SwerveSubsystem swerveSubsystem = SwerveSubsystem.getInstance();
	private final LightsSubsystem lightsSubsystem = LightsSubsystem.getInstance();

	/* State Flags */
	boolean requestHome = false;
	boolean requestIdle = false;

	/* Other Variables */
	private double mStateStartTime = 0.0;
	private SuperstructureState systemState = SuperstructureState.PRE_HOME;

	boolean homedOnce = true; // TODO: this should be false
	private double lastFPGATimestamp = 0.0;

	private Superstructure() {}

	public static Superstructure getInstance() {
		if (instance == null) instance = new Superstructure();
		return instance;
	}

	@Override
	public void periodic() {
		double time = RobotController.getFPGATime() / 1.0E6;
		SmartDashboard.putNumber("Superstructure/loopCycleTime", time - lastFPGATimestamp);
		lastFPGATimestamp = time;

		SmartDashboard.putString("Superstructure/Superstructure State", systemState.toString());
		SmartDashboard.putBoolean("Superstructure/Homed Once", homedOnce);

		SuperstructureState nextState = systemState;
		switch (systemState) {
			case PRE_HOME -> {}
			case IDLE -> {}
			default -> throw new IllegalArgumentException("Guess I missed a state");
		}

		if (nextState != systemState) {
			mStateStartTime = time;
			systemState = nextState;
		}
	}

	public void unsetAllRequests() {
		requestHome = false;
		requestIdle = false;
	}

	public void requestHome() {
		unsetAllRequests();
		requestHome = true;
	}

	public void requestIdle() {
		unsetAllRequests();
		requestIdle = true;
	}
}
