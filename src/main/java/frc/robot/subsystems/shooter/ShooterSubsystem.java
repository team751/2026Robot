package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.shooter.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {
private static ShooterSubsystem instance;

// Motors
private final TalonFX flywheelMotor = ShooterConstants.flywheelMotorConfig.createDevice(TalonFX::new);
private final VoltageOut flywheelControl = new VoltageOut(0);

private final TalonFX backMotor = ShooterConstants.backMotorConfig.createDevice(TalonFX::new);
private final VoltageOut backControl = new VoltageOut(0);



/* State Machine Logic */
private enum ShooterState {
	IDLE,
	SPINNING
}

private ShooterState state = ShooterState.IDLE;

private boolean requestedIdle = false;
private boolean requestedShoot = false;

public static ShooterSubsystem getInstance() {
	if (instance == null) instance = new ShooterSubsystem();
	return instance;
}

private ShooterSubsystem() {
	setShooterMotor(0,0);
}

@Override
public void periodic() {
	ShooterState nextState = state;
	if (requestedIdle) nextState = ShooterState.IDLE;
	else if (requestedShoot) nextState = ShooterState.SPINNING;

	if (nextState != state) {
	state = nextState;
	unsetAllRequests();

	switch (state) {
		case IDLE -> setShooterMotor(0,0);
		case SPINNING -> setShooterMotor(ShooterConstants.flywheelSpeed, ShooterConstants.backSpeed);
	}

	}

	SmartDashboard.putNumber("Shooter/Back Speed", backMotor.getVelocity().getValueAsDouble());
	SmartDashboard.putNumber("Shooter/Back Duty",backMotor.getDutyCycle().getValueAsDouble());

	SmartDashboard.putNumber("Shooter/Flywheel Speed", flywheelMotor.getVelocity().getValueAsDouble());
		SmartDashboard.putNumber("Shooter/Flywheel Duty", flywheelMotor.getDutyCycle().getValueAsDouble());
}

private void setShooterMotor(double voltage1, double voltage2) {
	flywheelMotor.setControl(flywheelControl.withOutput(voltage1));
	backMotor.setControl(backControl.withOutput(voltage2));
}

public void newShooterSpeed(double flySpeed, double backSpeed) {
	ShooterConstants.flywheelSpeed = flySpeed;
	ShooterConstants.backSpeed = backSpeed;
	this.requestShoot();
}

private void unsetAllRequests() {
	requestedIdle = false;
	requestedShoot = false;
}

public void requestIdle() {
	newShooterSpeed(0,0);

	unsetAllRequests();
	requestedIdle = true;
}

public void requestShoot() {
	unsetAllRequests();
	requestedShoot = true;
}
}