package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.CTREConfig;
import frc.robot.Robot;

import frc.robot.util.Constants;

public class ShooterConstants {
// TODO: Set current limits and tune the motors. also name/find the canbus to put the motors on
	public static double flywheelSpeed = 1; // 12
	public static double backSpeed = 1;

	public static final CTREConfig<TalonFX, TalonFXConfiguration> flywheelMotorConfig = 
		new CTREConfig<>(TalonFXConfiguration::new);

	public static final CTREConfig<TalonFX, TalonFXConfiguration> backMotorConfig = 
		new CTREConfig<>(TalonFXConfiguration::new);

	static {
		flywheelMotorConfig.withName("Flywheel Motor").withCanID(Constants.flywheelMotorID)/*.withBus(Robot.TBD)*/;

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

		// flywheelConfig.CurrentLimits.StatorCurrentLimit = 120;
		// flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = false;
		// flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		// flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
	}


	static {
		backMotorConfig.withName("Back Motor").withCanID(Constants.backMotorID)/*.withBus(Robot.TBD)*/;

		TalonFXConfiguration backConfig = backMotorConfig.config;

		backConfig.Slot0.kP = 0.1; // Increase until speed oscillates
		backConfig.Slot0.kI = 0; // Don't touch haha im gunna touch it lol
		backConfig.Slot0.kD = 0; // Increase until jitter

		backConfig.Slot0.kS = 0; // Increase until just before motor starts moving
		backConfig.Slot0.kA = 0; //
		backConfig.Slot0.kV = 0; //
		backConfig.Slot0.kG = 0; // Don't touch

		backConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
		backConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
		// backConfig.CurrentLimits.StatorCurrentLimit = 120;
		// backConfig.CurrentLimits.StatorCurrentLimitEnable = false;
		// backConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
		// backConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
	}

}
