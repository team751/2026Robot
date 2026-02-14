package frc.robot.subsystems.motor;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

public class Motor {
    private static Motor instance;

    public static TalonFX motor1 = new TalonFX(10);

    public static Motor getInstance() {
        if (instance == null) instance = new Motor();
        return instance;
    }

    private Motor() {}

    public static void runMotor(double speed) {
        motor1.setControl(new DutyCycleOut(speed));
    }

    public static  void stopMotor() {
        motor1.stopMotor();
    }
    
    
}
