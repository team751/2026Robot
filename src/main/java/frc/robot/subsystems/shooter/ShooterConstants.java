package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.CTREConfig;
import frc.robot.Robot;

public class ShooterConstants {
public static double flywheelSpeed = 1; // 12
public static double backSpeed = 1;

public static final CTREConfig<TalonFX, TalonFXConfiguration> flywheelMotorConfig = 
	new CTREConfig<>(TalonFXConfiguration::new);

	static {
		flywheelMotorConfig.withName("Flywheel Main").withCanID(11).withBus(Robot.gamepiecebus);

		TalonFXConfiguration flywheelConfig = flywheelMotorConfig.config;
		flywheelConfig.Slot0.kP = 0.1; // Increase until speed oscillates
		flywheelConfig.Slot0.kI = 0; // Don't touch
		flywheelConfig.Slot0.kD = 0; // Increase until jitter
		flywheelConfig.Slot0.kS = 0; // Increase until just before motor starts moving
		flywheelConfig.Slot0.kA = 0; //
		flywheelConfig.Slot0.kV = 0; //
		flywheelConfig.Slot0.kG = 0; // Don't touch

		flywheelConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
	}

public static final CTREConfig<TalonFX, TalonFXConfiguration> followMotorConfig = 
	new CTREConfig<>(TalonFXConfiguration::new);

static {
	followMotorConfig.withName("Flywheel Follow").withCanID(10).withBus(Robot.gamepiecebus);

	TalonFXConfiguration followConfig = followMotorConfig.config;

	followConfig.Slot0.kP = 0.1; // Increase until speed oscillates
	followConfig.Slot0.kI = 0; // Don't touch haha im gunna touch it lol
	followConfig.Slot0.kD = 0; // Increase until jitter
	followConfig.Slot0.kS = 0; // Increase until just before motor starts moving
	followConfig.Slot0.kA = 0; //
	followConfig.Slot0.kV = 0; //
	followConfig.Slot0.kG = 0; // Don't touch

	followConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
	followConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
}

}
