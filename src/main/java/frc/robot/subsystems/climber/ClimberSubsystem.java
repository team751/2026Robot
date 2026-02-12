package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
private final TalonFX leftClimber = new TalonFX(10);
private final TalonFX rightClimber = new TalonFX(11);

private final PositionVoltage positionRequest = new PositionVoltage(0);

private static ClimberSubsystem instance;

public static ClimberSubsystem getInstance() {
	if (instance == null) instance = new ClimberSubsystem();
	return instance;
}

private ClimberSubsystem() {
	TalonFXConfiguration config = new TalonFXConfiguration();
	config.Slot0.kP = 1.0;
	config.Slot0.kI = 0.0;
	config.Slot0.kD = 0.0;

	leftClimber.getConfigurator().apply(config);
	rightClimber.getConfigurator().apply(config);

	// Set neutral mode
	// dont invert
}

@Override
public void periodic() {

}

// write a methods (think of a function, ask chatgpt if u dont get it) that:
// moves the climber motors up to 180 degrees
// moves the climber motors down to 180 degrees
// stops the motors

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