// package frc.robot.subsystems.shooter;

// import com.ctre.phoenix6.StatusSignal;
// import com.ctre.phoenix6.controls.VoltageOut;
// import com.ctre.phoenix6.hardware.CANrange;
// import com.ctre.phoenix6.hardware.TalonFX;
// import edu.wpi.first.math.filter.LinearFilter;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.shooter.ShooterConstants;

// public class ShooterSubsystem extends SubsystemBase {
// private static ShooterSubsystem instance;

// /* Motors */
// private final TalonFX shooterMotor = ShooterConstants.shooterMotorConfig.createDevice(TalonFX::new);
// private final VoltageOut shooterControl = new VoltageOut(0);

// /* State Machine Logic */
// private enum ShooterState {
// 	IDLE,
// 	SPINNING
// }

// private ShooterState state = ShooterState.IDLE;

// private boolean requestedIdle = false;
// private boolean requestedShoot = false;

// public static ShooterSubsystem getInstance() {
// 	if (instance == null) instance = new ShooterSubsystem();
// 	return instance;
// }

// private ShooterSubsystem() {
// 	setShooterMotor(0);
// }

// /**
// * Set the intake motor to a given speed
// *
// * @param voltage in volts
// */
// private void setShooterMotor(double voltage) {
// 	shooterMotor.setControl(shooterControl.withOutput(voltage));
// }

// @Override
// public void periodic() {
// 	ShooterState nextState = state;
// 	if (requestedIdle) nextState = ShooterState.IDLE;
// 	else if (requestedShoot) nextState = ShooterState.SPINNING;

// 	if (nextState != state) {
// 	state = nextState;
// 	unsetAllRequests();

// 	switch (state) {
// 		case IDLE -> setShooterMotor(0);
// 		case SPINNING -> setShooterMotor(ShooterConstants.shooterSpeed);
// 	}

// 	}

// 	SmartDashboard.putString("Intake/Intake State", state.toString());
// 	// SmartDashboard.putBoolean("Intake/Stalled", stalled);
// 	SmartDashboard.putNumber("Intake/Intake Speed", shooterMotor.getVelocity().getValueAsDouble());
// }


// private void unsetAllRequests() {
// 	requestedIdle = false;
// 	requestedShoot = false;
// }

// public void requestIdle() {
// 	unsetAllRequests();
// 	requestedIdle = true;
// }

// public void requestShoot() {
// 	unsetAllRequests();
// 	requestedShoot = true;
// }
// }