package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimbSubsystem extends SubsystemBase {
	private static ClimbSubsystem instance;

	public static ClimbSubsystem getInstance() {
		if (instance == null) instance = new ClimbSubsystem();
		return instance;
	}

	private ClimbSubsystem() {
		// pivotMotor.setControl(pivotControl);
	}

	@Override
	public void periodic() {}
}
