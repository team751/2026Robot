package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.CTREConfig;
import frc.robot.Robot;

public class ClimberConstants {
public static double extendSpeed = 0.6;
public static double retractSpeed = -0.6; 

public static final CTREConfig<TalonFX, TalonFXConfiguration> climberMotorConfig = 
	new CTREConfig<>(TalonFXConfiguration::new);

	static {
		climberMotorConfig.withName("Climber Motor").withCanID(11).withBus(Robot.riobus);
		TalonFXConfiguration climberConfig = climberMotorConfig.config;
		climberConfig.Slot0.kP = 0.1; // Increase until speed oscillates
		climberConfig.Slot0.kI = 0; // Don't touch
		climberConfig.Slot0.kD = 0; // Increase until jitter
		climberConfig.Slot0.kS = 0; // Increase until just before motor starts moving
		climberConfig.Slot0.kA = 0; //
		climberConfig.Slot0.kV = 0; //
		climberConfig.Slot0.kG = 0; // Don't touch

		climberConfig.CurrentLimits.StatorCurrentLimit = 120;
		climberConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		climberConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		climberConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

		climberConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
		climberConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
	}

public static final CTREConfig<TalonFX, TalonFXConfiguration> backMotorConfig = 
	new CTREConfig<>(TalonFXConfiguration::new);

static {
	backMotorConfig.withName("Back Motor").withCanID(10).withBus(Robot.riobus);

	TalonFXConfiguration backConfig = backMotorConfig.config;

	backConfig.Slot0.kP = 0.1; // Increase until speed oscillates
	backConfig.Slot0.kI = 0; // Don't touch haha im gunna touch it lol
	backConfig.Slot0.kD = 0; // Increase until jitter
	backConfig.Slot0.kS = 0; // Increase until just before motor starts moving
	backConfig.Slot0.kA = 0; //
	backConfig.Slot0.kV = 0; //
	backConfig.Slot0.kG = 0; // Don't touch

	backConfig.CurrentLimits.StatorCurrentLimit = 120;
	backConfig.CurrentLimits.StatorCurrentLimitEnable = true;
	backConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
	backConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

	backConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
	backConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
}

}
