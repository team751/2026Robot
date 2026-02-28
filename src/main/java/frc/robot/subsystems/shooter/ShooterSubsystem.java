package frc.robot.subsystems.shooter;

import java.util.function.Consumer;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class ShooterSubsystem extends SubsystemBase {
private static ShooterSubsystem instance;

// Motors
private final TalonFX flywheelMotor = ShooterConstants.flywheelMotorConfig.createDevice(TalonFX::new);
private final VoltageOut flywheelControl = new VoltageOut(0);

// private final TalonFX backMotor = ShooterConstants.backMotorConfig.createDevice(TalonFX::new);
// private final VoltageOut backControl = new VoltageOut(0);


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

/**
* Set the intake motor to a given speed
*
* @param voltage1 in volts
 * @param voltage2 in volts
*/
private void setShooterMotor(double voltage1, double voltage2) {
	flywheelMotor.setControl(flywheelControl.withOutput(voltage1));
	//backMotor.setControl(backControl.withOutput(voltage2));
}

public void stopShooter() {
	flywheelMotor.setControl(flywheelControl.withOutput(0));
	SignalLogger.stop();
}

private SysIdRoutine routine = new SysIdRoutine(new SysIdRoutine.Config(), new SysIdRoutine.Mechanism(
                    volts -> flywheelMotor.setControl(
                        flywheelControl.withOutput(volts)
                    ),
                    null,
                    this));


public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
	SignalLogger.start();
  return routine.quasistatic(direction);
}

public Command sysIdDynamic(SysIdRoutine.Direction direction) {
	SignalLogger.start();
  return routine.dynamic(direction);
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

	SmartDashboard.putString("Shooter/Shoot State", state.toString());

	SmartDashboard.putNumber("Shooter/Flywheel Speed", flywheelMotor.getVelocity().getValueAsDouble());
		SmartDashboard.putNumber("Shooter/Flywheel Duty", flywheelMotor.getDutyCycle().getValueAsDouble());
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
	unsetAllRequests();
	requestedIdle = true;

	stopShooter();
}

public void requestShoot() {
	unsetAllRequests();
	requestedShoot = true;
}
}