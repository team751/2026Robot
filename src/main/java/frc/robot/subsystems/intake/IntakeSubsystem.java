package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lights.LightsSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
	private static IntakeSubsystem instance;

private final TalonFX intakeMotor = IntakeConstants.intakeMotorConfig.createDevice(TalonFX::new);
private final TalonFX extenderMotor = IntakeConstants.extenderMotorConfig.createDevice(TalonFX::new);
	/* Control Signals */
private final VoltageOut intakeControl = new VoltageOut(0);
private final VoltageOut extenderControl = new VoltageOut(0);

	/* State Machine Logic */
	private enum IntakeState {
		IDLE,
		INTAKING,
		SPITTING,
		EXTENDING,
		RETRACTING,
	}

	private IntakeState state = IntakeState.IDLE;

	private boolean requestedIdle = false;
	private boolean requestedIntaking = false;
	private boolean requestedSpitting = false;
	private boolean requestedExtending = false;
	private boolean requestedRetracting = false;

public static IntakeSubsystem getInstance() {
	if (instance == null) instance = new IntakeSubsystem();
	return instance;
}

private IntakeSubsystem() {
	setIntakeMotor(0);
	setExtenderMotor(0);
}

/**
* Set the intake motor to a given speed
*
* @param voltage in volts
*/
private void setIntakeMotor(double voltage) {
	intakeMotor.setControl(intakeControl.withOutput(voltage));
}
private void setExtenderMotor(double voltage) {
	extenderMotor.setControl(extenderControl.withOutput(voltage));
}

	@Override
	public void periodic() {
		IntakeState nextState = state;
		if (requestedIdle) nextState = IntakeState.IDLE;
		else if (requestedIntaking) nextState = IntakeState.INTAKING;
		else if (requestedSpitting) nextState = IntakeState.SPITTING;
		else if (requestedExtending) nextState = IntakeState.EXTENDING;
		else if (requestedRetracting) nextState = IntakeState.RETRACTING;

	if (nextState != state) {
		state = nextState;
		unsetAllRequests();

		switch (state) {
		case IDLE -> {
		setIntakeMotor(0);
		}
		case INTAKING -> setIntakeMotor(IntakeConstants.intakeSpeed);
		case SPITTING -> setIntakeMotor(IntakeConstants.spitSpeed);
		case EXTENDING -> setExtenderMotor(IntakeConstants.extenderSpeed);
		case RETRACTING -> setExtenderMotor(IntakeConstants.retractorSpeed);
	}
	LightsSubsystem.getInstance().requestBlinking(state != IntakeState.IDLE);
	SmartDashboard.putString("Intake/Intake State", state.toString());
	SmartDashboard.putNumber("Intake/Intake Speed", intakeMotor.getVelocity().getValueAsDouble());
	SmartDashboard.putNumber("Extender/Extender Speed", extenderMotor.getVelocity().getValueAsDouble());
	}
}

	public void requestIntaking() {
		unsetAllRequests();
		requestedIntaking = true;
	}
	public void requestIdle() {
		unsetAllRequests();
		requestedIdle = true;
	}
	public void requestSpitting() {
		unsetAllRequests();
		requestedSpitting = true;
	}
	public void requestExtending() {
		unsetAllRequests();
		requestedExtending = true;
	}
	public void requestRetracting() {
		unsetAllRequests();
		requestedRetracting = true;
	}
	private void unsetAllRequests() {
	requestedIdle = false;
	requestedIntaking = false;
	requestedSpitting = false;
	requestedExtending = false;
	requestedRetracting = false;
}
}