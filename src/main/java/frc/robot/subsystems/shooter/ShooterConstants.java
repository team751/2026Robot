// package frc.robot.subsystems.shooter;

// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.hardware.TalonFX;
// import com.ctre.phoenix6.signals.InvertedValue;
// import frc.lib.CTREConfig;
// import frc.robot.Robot;

// public class ShooterConstants {
// public static final double shooterSpeed = 12;
// public static final double spitSpeed = 4; // Negated in request
// //    public static final int stalledCurrentThreshold = 30;

// public static final CTREConfig<TalonFX, TalonFXConfiguration> shooterMotorConfig =
// 	new CTREConfig<>(TalonFXConfiguration::new);

// static {
// 	shooterMotorConfig.withName("Shooter Motor").withCanID(15).withBus(Robot.drivebus);

// 	TalonFXConfiguration shooterConfig = shooterMotorConfig.config;
// 	shooterConfig.Slot0.kP = 0; // Increase until speed oscillates
// 	shooterConfig.Slot0.kI = 0; // Don't touch haha im gunna touch it lol
// 	shooterConfig.Slot0.kD = 0; // Increase until jitter
// 	shooterConfig.Slot0.kS = 0; // Increase until just before motor starts moving
// 	shooterConfig.Slot0.kA = 0; //
// 	shooterConfig.Slot0.kV = 0; //
// 	shooterConfig.Slot0.kG = 0; // Don't touch

// 	shooterConfig.Feedback.RotorToSensorRatio = 1;
// 	shooterConfig.Feedback.SensorToMechanismRatio = 1;

// 	shooterConfig.CurrentLimits.StatorCurrentLimit = 120;
// 	shooterConfig.CurrentLimits.StatorCurrentLimitEnable = false;

// 	shooterConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
// 	shooterConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;

// 	shooterConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive; // Positive shooter
// }
// }
