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
  public static double flywheelSpeed = 27; // Rotations per Second
  public static double transferVoltage = 2; // Volts
  public static double slowPercent = 0.5; // Percent 0.0-1.0
  public static double transferSpitVoltage = -2;

  public static final double shooterDistanceCurveYIntercept = -420; // RPM at 0 distance
  public static final double shooterDistanceCurveSlope = 16.8; // RPM per cm
  public static final double minShootingDistance =
      50.0; // cm, distance at which to use minimum shooter speed
  public static final double maxShootingDistance =
      600.0; // cm, distance at which to use maximum shooter speed

  public static final CTREConfig<TalonFX, TalonFXConfiguration> flywheelMotorConfig =
      new CTREConfig<>(TalonFXConfiguration::new);

  public static final CTREConfig<TalonFX, TalonFXConfiguration> followMotorConfig =
      new CTREConfig<>(TalonFXConfiguration::new);

  public static final CTREConfig<TalonFX, TalonFXConfiguration> transferMotorConfig =
      new CTREConfig<>(TalonFXConfiguration::new);

  static {
    flywheelMotorConfig
        .withName("Main Flywheel")
        .withCanID(Constants.flywheelMotorID)
        .withBus(Robot.gamepiecebus);

    TalonFXConfiguration flywheelConfig = flywheelMotorConfig.config;
    flywheelConfig.Slot0.kP = 0.1; // Increase until speed oscillates
    flywheelConfig.Slot0.kI = 0; // Don't touch
    flywheelConfig.Slot0.kD = 0; // Increase until jitter

    flywheelConfig.Slot0.kS = 0; // Increase until just before motor starts moving
    flywheelConfig.Slot0.kA = 0; //
    flywheelConfig.Slot0.kV = 0.13; //
    flywheelConfig.Slot0.kG = 0; // Don't touch

    flywheelConfig.MotorOutput.ControlTimesyncFreqHz = 75;

    flywheelConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    flywheelConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // flywheelConfig.CurrentLimits.StatorCurrentLimit = 120;
    // flywheelConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    // flywheelConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    // flywheelConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
  }

  static {
    followMotorConfig
        .withName("Follow Flywheel")
        .withCanID(Constants.followMotorID)
        .withBus(Robot.gamepiecebus);

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
    // followConfig.CurrentLimits.StatorCurrentLimit = 120;
    // followConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    // followConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    // followConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
  }

  static {
    transferMotorConfig
        .withName("Shooter Transfer")
        .withCanID(Constants.shooterTransferMotorID)
        .withBus(Robot.gamepiecebus);

    TalonFXConfiguration shooterTransferConfig = transferMotorConfig.config;

    shooterTransferConfig.Slot0.kP = 0.1;
    shooterTransferConfig.Slot0.kI = 0.0;
    shooterTransferConfig.Slot0.kD = 0.0;

    shooterTransferConfig.Slot0.kS = 0.0;
    shooterTransferConfig.Slot0.kA = 0.0;
    shooterTransferConfig.Slot0.kV = 0.0;
    shooterTransferConfig.Slot0.kG = 0.0;

    shooterTransferConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    shooterTransferConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // just copy from above
  }
}
