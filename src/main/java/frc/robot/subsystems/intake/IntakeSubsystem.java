package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	private static IntakeSubsystem instance;

	/* State Machine Logic */
	private enum IntakeState {
		IDLE
	}

	private IntakeState state = IntakeState.IDLE;

	private boolean requestedIdle = false;

	public static IntakeSubsystem getInstance() {
		if (instance == null) instance = new IntakeSubsystem();
		return instance;
	}

	private IntakeSubsystem() {}

	@Override
	public void periodic() {}
}
