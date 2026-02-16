package frc.robot.subsystems.motor;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorConstants {
    public static final TalonFX motor1 = new TalonFX(10);

    private static TalonFXConfiguration motor1Config = new TalonFXConfiguration();

    static {
       motor1Config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        motor1Config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
        motor1Config.Slot0.kP = 20.0;
        motor1Config.Slot0.kI = 1; // Scary spooky value dont touch it
        motor1Config.Slot0.kD = 5;

        motor1Config.Slot0.kS = 0.0;
        motor1Config.Slot0.kV = 0.0;
        motor1Config.Slot0.kG = 0.0;
        motor1Config.Slot0.kA = 0.0;

        motor1Config.CurrentLimits.SupplyCurrentLimitEnable = false;
        motor1Config.CurrentLimits.StatorCurrentLimitEnable = false;
        
    }

}