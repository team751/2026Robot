package frc.robot.subsystems.motor;

import org.ironmaple.simulation.Goal.PositionChecker;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Motor extends SubsystemBase {
    private static Motor instance;

    public static TalonFX motor1 = MotorConstants.motor1;

    private static boolean running = false;

    public static Motor getInstance() {
        if (instance == null) instance = new Motor();
        return instance;
    }

    private Motor() {}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Motor Pos",motor1.getPosition().getValueAsDouble());
    }

    public static void runMotor(double position) {
        SmartDashboard.putNumber("Target Pos (Rot)",Units.degreesToRotations(position));
        SmartDashboard.putNumber("Target Pos (Deg)",position);

        running = true;
        if (running) {
            motor1.setControl(new PositionVoltage(Units.degreesToRotations(position)).withSlot(0));
        }
    }

    public static void constantSpin() {
        running = true;
        if (running) {
            motor1.setControl(new DutyCycleOut(0.1));
        }
    }

    public static  void stopMotor() {
        motor1.stopMotor();
        running = false;
    }
    
    
}
