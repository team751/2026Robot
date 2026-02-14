package frc.robot.subsystems.limit_switch;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.motor.Motor;

public class LimitSwitch extends SubsystemBase {

private static LimitSwitch instance;

private static DigitalInput limitSwitch = new DigitalInput(0);

public static LimitSwitch getInstance() {
	if (instance == null) instance = new LimitSwitch();
	return instance;
}

private LimitSwitch() {}

@Override
public void periodic() {
	SmartDashboard.putBoolean("Pressed",limitSwitch.get());
}

public static boolean getSwitch() {
	return limitSwitch.get();
}

}