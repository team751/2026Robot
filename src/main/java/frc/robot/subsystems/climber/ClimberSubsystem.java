package frc.robot.subsystems.climber;

import org.dyn4j.dynamics.Torque;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ClimberSubsystem extends SubsystemBase {
private final TalonFX leftClimber = new TalonFX(10);
private final TalonFX rightClimber = new TalonFX(11);

private final PositionVoltage positionRequest = new PositionVoltage(0);

private static ClimberSubsystem instance;

// State used to run a non-blocking "spin until position" behavior
private boolean m_spinUntilTarget = false;
private double m_spinTarget = 10.0;

public static ClimberSubsystem getInstance() {
	if (instance == null) instance = new ClimberSubsystem();
	return instance;
}

private ClimberSubsystem() {
	TalonFXConfiguration config = new TalonFXConfiguration();
	config.Slot0.kP = 1;
	config.Slot0.kI = 0.0;
	config.Slot0.kD = 0.3;
	

	leftClimber.getConfigurator().apply(config);
	rightClimber.getConfigurator().apply(config);

	// Register so periodic() will be called by the CommandScheduler
	CommandScheduler.getInstance().registerSubsystem(this);

	// Set neutral mode
	// dont invert
}

@Override
public void periodic() {
	SmartDashboard.putNumber("Left Motor Pos", leftClimber.getPosition().getValueAsDouble());
	SmartDashboard.putNumber("Right Motor Pos", rightClimber.getPosition().getValueAsDouble());

	// If requested, run the spin-until-target logic non-blocking so callers don't have to loop.
	if (m_spinUntilTarget) {
		double pos = leftClimber.getPosition().getValueAsDouble();
		SmartDashboard.putNumber("Target", m_spinTarget);
		if (pos >= m_spinTarget) {
			stopMotors();
			m_spinUntilTarget = false;
			SmartDashboard.putBoolean("Stopped", true);
		} else {
			SmartDashboard.putBoolean("Stopped", false);
		}
	}

}

// write a methods (think of a function, ask chatgpt if u dont get it) that:
// moves the climber motors up to 180 degrees
// moves the climber motors down to 180 degrees
// stops the motors

public void moveVerySlowly() {
		leftClimber.setVoltage(0.5);
		rightClimber.setVoltage(0.5);
}

public void motorSync() {
	leftClimber.setControl(new DutyCycleOut(0.1));
	rightClimber.setControl(new Follower(10, MotorAlignmentValue.Aligned));
}

public void spinUntil10() {
	// Start a non-blocking spin that will stop when the left encoder reaches 10 rotations.
	m_spinTarget = 10.0;
	m_spinUntilTarget = true;
	leftClimber.setControl(new DutyCycleOut(0.1));
	rightClimber.setControl(new Follower(10, MotorAlignmentValue.Aligned));
}

/** Cancel an ongoing spin-until operation (if any) and stop the motors. */
public void cancelSpinUntil() {
	m_spinUntilTarget = false;
	stopMotors();
}

public void moveUp180(){

	double currentPosition = leftClimber.getPosition().getValueAsDouble();
	double targetPosition = currentPosition + 1.0;

	leftClimber.setControl(positionRequest.withPosition(targetPosition));
	rightClimber.setControl(positionRequest.withPosition(targetPosition));

}

public void moveDown180(){

	double currentPosition = leftClimber.getPosition().getValueAsDouble();
	double targetPosition = currentPosition - 1.0;

	leftClimber.setControl(positionRequest.withPosition(targetPosition));
	rightClimber.setControl(positionRequest.withPosition(targetPosition));

}

public void stopMotors(){
	leftClimber.stopMotor();
	rightClimber.stopMotor();
}

public void zeroClimber(){
	leftClimber.setPosition(0);
	rightClimber.setPosition(0);
}
// Alias for existing method name some callers may expect: start spinning until 10 rotations then stop.
public void stopUntil10() {
    spinUntil10();
}

}