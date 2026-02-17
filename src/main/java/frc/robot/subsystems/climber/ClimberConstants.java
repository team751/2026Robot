package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.CTREConfig;
import frc.robot.Robot;

public class ClimberConstants {
    public static double averageMotorError = 0.3; // Starting average error, it will change later


    public static final TalonFX leftClimber = new TalonFX(50);
    public static final TalonFX rightClimber = new TalonFX(51);

    private static TalonFXConfiguration leftClimberConfig = new TalonFXConfiguration();
    private static TalonFXConfiguration rightClimberConfig = new TalonFXConfiguration();

    static {
       leftClimberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leftClimberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
        leftClimberConfig.Slot0.kP = 0.0;
        leftClimberConfig.Slot0.kI = 0.0; // Scary spooky value dont touch it
        leftClimberConfig.Slot0.kD = 0.0;

        leftClimberConfig.Slot0.kS = 0.0;
        leftClimberConfig.Slot0.kV = 0.0;
        leftClimberConfig.Slot0.kG = 0.0;
        leftClimberConfig.Slot0.kA = 0.0;
    }

    static {
//      Could be important, was apart of something before
// 		flywheelMotorConfig.withName("Flywheel Motor").withCanID(11).withBus(Robot.riobus);
        rightClimberConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        rightClimberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        rightClimberConfig.Slot0.kP = 0.0;
        rightClimberConfig.Slot0.kI = 0.0; // Scary spooky value dont touch it
        rightClimberConfig.Slot0.kD = 0.0;

        rightClimberConfig.Slot0.kS = 0.0;
        rightClimberConfig.Slot0.kV = 0.0;
        rightClimberConfig.Slot0.kG = 0.0;
        rightClimberConfig.Slot0.kA = 0.0;
    }

}
