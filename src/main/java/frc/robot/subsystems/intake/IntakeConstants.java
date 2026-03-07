package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.CTREConfig;
import frc.robot.Robot;
import frc.robot.util.Constants;

public class IntakeConstants {
  // TODO: Set current limits and tune the motors. also name/find the canbus to put the motors on

  public static final double intakeSpeed = 3.5;
  public static final double spitSpeed = -3.5;
  public static final double extenderSpeed = 2;
  public static final double retractorSpeed = -2;
  public static final double jiggleMax = 28;
  public static final double jiggleMin = 5;
  
  public static final int FrontLeftLimitID = 0;
  public static final int BackLeftLimitID = 1;
  public static final int FrontRightLimitID = 2;
  public static final int BackRightLimitID = 3;

  public static final double extenderLength = 32.385; // cm
  public static final double extenderGearRatio = 2.73; //cm per rotation
  
  public static final CTREConfig<TalonFX, TalonFXConfiguration> intakeMotorConfig =
      new CTREConfig<>(TalonFXConfiguration::new);

  public static final CTREConfig<TalonFX, TalonFXConfiguration> extenderMotorConfig =
      new CTREConfig<>(TalonFXConfiguration::new);

  static {
    intakeMotorConfig
        .withName("Intake Motor")
        .withCanID(Constants.intakeMotorID)
        .withBus(Robot.gamepiecebus);

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

  static {
    extenderMotorConfig
        .withName("Extender Motor")
        .withCanID(Constants.extenderMotorID)
        .withBus(Robot.gamepiecebus);

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

    extenderConfig.MotorOutput.Inverted =
        InvertedValue.Clockwise_Positive; // Positive intake
    extenderConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
  }
}
