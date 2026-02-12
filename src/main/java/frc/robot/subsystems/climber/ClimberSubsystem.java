package frc.robot.subsystems.climber;


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
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class ClimberSubsystem extends SubsystemBase {

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
	// TalonFXConfiguration config = new TalonFXConfiguration();
	// config.Slot0.kP = 1;
	// config.Slot0.kI = 0.0;
	// config.Slot0.kD = 0.3;

	// config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
	

	// leftClimber.getConfigurator().apply(config);
	// rightClimber.getConfigurator().apply(config);


	//CommandScheduler.getInstance().registerSubsystem(this);
}

@Override
public void periodic() {
	SmartDashboard.putNumber("Left Motor Pos", ClimberConstants.leftClimber.getPosition().getValueAsDouble());
	SmartDashboard.putNumber("Right Motor Pos", ClimberConstants.rightClimber.getPosition().getValueAsDouble());

	if (m_spinUntilTarget) {
		SmartDashboard.putNumber("SpinUntil/Target", m_spinTarget);
		if (ClimberConstants.leftClimber.getPosition().getValueAsDouble() >= m_spinTarget) {
			stopMotors();
			m_spinUntilTarget = false;
			SmartDashboard.putBoolean("SpinUntil/Stopped", true);
		} else {
			SmartDashboard.putBoolean("SpinUntil/Stopped", false);
		}
	}

}


public void moveVerySlowly() {
		ClimberConstants.leftClimber.setVoltage(0.5);
		ClimberConstants.rightClimber.setVoltage(0.5);
}

public void motorSync() {
	ClimberConstants.leftClimber.setControl(new DutyCycleOut(0.1));
	ClimberConstants.rightClimber.setControl(new Follower(10, MotorAlignmentValue.Aligned));
}

public void spinUntil10() {
	m_spinTarget = 10.0;
	m_spinUntilTarget = true;
	if (m_spinUntilTarget) {
		ClimberConstants.leftClimber.setControl(new DutyCycleOut(0.1));
		ClimberConstants.rightClimber.setControl(new Follower(10, MotorAlignmentValue.Aligned));
	}
}

public void cancelSpinUntil() {
	m_spinUntilTarget = false;
	stopMotors();
}

public void moveUp180(){

	double currentPosition = ClimberConstants.leftClimber.getPosition().getValueAsDouble();
	double targetPosition = currentPosition + 1.0;

	ClimberConstants.leftClimber.setControl(positionRequest.withPosition(targetPosition));
	ClimberConstants.rightClimber.setControl(positionRequest.withPosition(targetPosition));

}

public void moveDown180(){

	double currentPosition = ClimberConstants.leftClimber.getPosition().getValueAsDouble();
	double targetPosition = currentPosition - 1.0;

	ClimberConstants.leftClimber.setControl(positionRequest.withPosition(targetPosition));
	ClimberConstants.rightClimber.setControl(positionRequest.withPosition(targetPosition));

}

public void stopMotors(){
	ClimberConstants.leftClimber.stopMotor();
	ClimberConstants.rightClimber.stopMotor();
}

public void zeroClimber(){
	ClimberConstants.leftClimber.setPosition(0);
	ClimberConstants.rightClimber.setPosition(0);
}


}