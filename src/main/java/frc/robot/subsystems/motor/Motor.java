package frc.robot.subsystems.motor;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.limit_switch.LimitSwitch;

public class Motor extends SubsystemBase {
    private static Motor instance;

    public static TalonFX motor1 = new TalonFX(10);

    private static boolean running = false;

    public static Motor getInstance() {
        if (instance == null) instance = new Motor();
        return instance;
    }

    private Motor() {}

    @Override
    public void periodic() {
        if (running && LimitSwitch.getSwitch()) {
            stopMotor();
        }
    }

    public static void runMotor(double speed) {
        running = true;
        if (running) {
            motor1.setControl(new DutyCycleOut(speed));
        }
    }

    public static  void stopMotor() {
        motor1.stopMotor();
        running = false;
    }
    
    
}
