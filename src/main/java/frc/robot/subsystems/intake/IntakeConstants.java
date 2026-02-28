package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.lib.CTREConfig;
import frc.robot.Robot;

public class IntakeConstants {
	// Lucas remember this is in volts. This is wayyyy to fast. 4, 6, & 12v of power is insane .-.
	public static final double intakeSpeed = 12; // Prolly more like 4-5v (MAYBE 6v if we need it)
	public static final double spitSpeed = 4; // 4 is fine, but 2.5v or 3v would take less power
	public static final double extenderSpeed = 6; // Way way WAYY to fast. Its going to apply (and waste) a ton of power
	public static final double retractorSpeed = -6; // Something more like 2v would be good

    public static final CTREConfig<TalonFX, TalonFXConfiguration> intakeMotorConfig =
	new CTREConfig<>(TalonFXConfiguration::new);

    static { // Climber and swerve get their own bus's, so whatever we end up naming the last bus will be used for everything but climber and swerve
		//TODO: Find CAN ID and bus for intake motor
	//intakeMotorConfig.withName("Intake Motor").withCanID("TBS").withBus("TBD");

	TalonFXConfiguration intakeConfig = intakeMotorConfig.config;
	intakeConfig.Slot0.kP = 0; // Increase until speed oscillates
	intakeConfig.Slot0.kI = 0; // Don't touch
	intakeConfig.Slot0.kD = 0; // Increase until jitter
	intakeConfig.Slot0.kS = 0; // Increase until just before motor starts moving
	intakeConfig.Slot0.kA = 0; //
	intakeConfig.Slot0.kV = 0; //
	intakeConfig.Slot0.kG = 0; // Don't touch

	intakeConfig.Feedback.RotorToSensorRatio = 1;
	intakeConfig.Feedback.SensorToMechanismRatio = 1;

	intakeConfig.CurrentLimits.StatorCurrentLimit = 120;
	intakeConfig.CurrentLimits.StatorCurrentLimitEnable = false;
	intakeConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
	intakeConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

	intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Positive intake
	intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
}




public static final CTREConfig<TalonFX, TalonFXConfiguration> extenderMotorConfig =
	new CTREConfig<>(TalonFXConfiguration::new);
    
    static {
	extenderMotorConfig.withName("Extender Motor").withCanID(52)/*.withBus("TBD") */;

	TalonFXConfiguration extenderConfig = extenderMotorConfig.config;
	extenderConfig.Slot0.kP = 0; // Increase until speed oscillates
	extenderConfig.Slot0.kI = 0; // Don't touch
	extenderConfig.Slot0.kD = 0; // Increase until jitter
	extenderConfig.Slot0.kS = 0; // Increase until just before motor starts moving
	extenderConfig.Slot0.kA = 0; //
	extenderConfig.Slot0.kV = 0; //
	extenderConfig.Slot0.kG = 0; // Don't touch
	extenderConfig.Feedback.RotorToSensorRatio = 1;
	extenderConfig.Feedback.SensorToMechanismRatio = 1;

	extenderConfig.CurrentLimits.StatorCurrentLimit = 120;
	extenderConfig.CurrentLimits.StatorCurrentLimitEnable = false;
	extenderConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
	extenderConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

	extenderConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Positive intake
	extenderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
}
}
