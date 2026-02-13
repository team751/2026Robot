package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.DutyCycleOut;
// import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
private final TalonFX leftClimber = new TalonFX(10);
private final TalonFX rightClimber = new TalonFX(11);

private final PositionVoltage positionRequest = new PositionVoltage(0);

private static ClimberSubsystem instance;

// State used to run a non-blocking "spin until position" behavior
private boolean spinning = false;
private double spinTarget = 0.0;

private boolean inverted = false;

public static ClimberSubsystem getInstance() {
	if (instance == null) instance = new ClimberSubsystem();
	return instance;
}

private ClimberSubsystem() {}

@Override
public void periodic() {
	SmartDashboard.putNumber("Left Motor Pos", ClimberConstants.leftClimber.getPosition().getValueAsDouble());
	SmartDashboard.putNumber("Right Motor Pos", ClimberConstants.rightClimber.getPosition().getValueAsDouble());

	// Spin to target check
	if (spinning) {
		SmartDashboard.putNumber("SpinUntil/Target", spinTarget);

		// If its not inverted, check if the value of the motor is greater than or equal to the target and then
		if (!inverted && ClimberConstants.leftClimber.getPosition().getValueAsDouble() >= spinTarget) {
			// stop the motors and set the spinning value to false
			stopSpinUntil();
			spinning = false;
		}

		// If it is inverted, check if the value of the motor is less than or equal to the target and then 
		if (inverted && ClimberConstants.leftClimber.getPosition().getValueAsDouble() <= spinTarget) {
			// stop the motors and set the spinning value to false.
			stopSpinUntil();
			spinning = false;
		}	
	}
}


public void moveVerySlowly() {
		ClimberConstants.leftClimber.setVoltage(0.5);
		ClimberConstants.rightClimber.setVoltage(0.5);
}

public void spinUntil(double value) {
	// Error and setup
	spinTarget = value - ClimberConstants.averageMotorError;
	spinning = true;

	if (spinning) {
		// Inversion check
		if (ClimberConstants.leftClimber.getPosition().getValueAsDouble() > spinTarget) {
			inverted = true;

			// Set the motors to spin
			ClimberConstants.leftClimber.setControl(new DutyCycleOut(-0.1));
			ClimberConstants.rightClimber.setControl(new StrictFollower(10));
		} else {
			inverted = false;

			// Set the motors to spin
			ClimberConstants.leftClimber.setControl(new DutyCycleOut(0.1));
			ClimberConstants.rightClimber.setControl(new StrictFollower(10));
		}
		
	}
}

public void stopSpinUntil() {
	spinning = false;

	// Get the average error
	double error = spinTarget - ClimberConstants.leftClimber.getPosition().getValueAsDouble();
	ClimberConstants.averageMotorError = (ClimberConstants.averageMotorError + error) / 3;

	stopMotors();
}

public void moveUp180(){

	double currentPosition = leftClimber.getPosition().getValueAsDouble();
	double targetPosition = currentPosition + 0.6;

	leftClimber.setControl(positionRequest.withPosition(targetPosition));
	rightClimber.setControl(positionRequest.withPosition(targetPosition));

}

public void moveDown180(){

	double currentPosition = leftClimber.getPosition().getValueAsDouble();
	double targetPosition = currentPosition - 0.6;

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

public void moveUpManual(){
	leftClimber.set(0.3);
	rightClimber.set(0.3);
}

public void moveDownManual(){
	leftClimber.set(-0.3);
	rightClimber.set(-0.3);
}
}