package frc.robot.subsystems.limit_switch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.motor.Motor;

public class LimitSwitch extends SubsystemBase {

private static LimitSwitch instance;

DigitalInput limitSwitch = new DigitalInput(0);

public static LimitSwitch getInstance() {
	if (instance == null) instance = new LimitSwitch();
	return instance;
}

private LimitSwitch() {}

@Override
public void periodic() {
	if (limitSwitch.get()) {
		Motor.runMotor(0.1);
	} else {
		Motor.stopMotor(); 
	}

	SmartDashboard.putBoolean("Pressed",limitSwitch.get());
}

}