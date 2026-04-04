package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.lib.CTREConfig;
import frc.robot.Robot;
import frc.robot.util.Constants;

public class TransferConstants {
  // TODO: Set current limits and tune the motors. also name/find the canbus to put the motors on
  public static double transfertopspeed = 6;
  public static double transferbottomspeed = 9;

  public static final CTREConfig<TalonFX, TalonFXConfiguration> transfertopconfig =
      new CTREConfig<>(TalonFXConfiguration::new);

  public static final CTREConfig<TalonFX, TalonFXConfiguration> transferbottomconfig =
      new CTREConfig<>(TalonFXConfiguration::new);

  static {
    transfertopconfig
        .withName("Transfer Top Motor")
        .withCanID(Constants.rightTransferMotorID)
        .withBus(Robot.riobus);

    TalonFXConfiguration transfertopConfig = transfertopconfig.config;
    transfertopConfig.Slot0.kP = 0.1; // Increase until speed oscillates
    transfertopConfig.Slot0.kI = 0; // Don't touch
    transfertopConfig.Slot0.kD = 0; // Increase until jitter

    transfertopConfig.Slot0.kS = 0; // Increase until just before motor starts moving
    transfertopConfig.Slot0.kA = 0; //
    transfertopConfig.Slot0.kV = 0; //
    transfertopConfig.Slot0.kG = 0; // Don't touch

    transfertopConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    transfertopConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    // transfertopConfig.CurrentLimits.StatorCurrentLimit = 120;
    // transfertopConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    // transfertopConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    // transfertopConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
  }

  static {
    transferbottomconfig
        .withName("Transfer Bottom Motor")
        .withCanID(Constants.leftTransferMotorID)
        .withBus(Robot.riobus);

    TalonFXConfiguration transferbottomConfig = transferbottomconfig.config;

    transferbottomConfig.Slot0.kP = 0.1; // Increase until speed oscillates
    transferbottomConfig.Slot0.kI = 0; // Don't touch haha im gunna touch it lol
    transferbottomConfig.Slot0.kD = 0; // Increase until jitter

    transferbottomConfig.Slot0.kS = 0; // Increase until just before motor starts moving
    transferbottomConfig.Slot0.kA = 0; //
    transferbottomConfig.Slot0.kV = 0; //
    transferbottomConfig.Slot0.kG = 0; // Don't touch

    transferbottomConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    transferbottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    // transferbottomConfig.CurrentLimits.StatorCurrentLimit = 120;
    // transferbottomConfig.CurrentLimits.StatorCurrentLimitEnable = false;
    // transferbottomConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    // transferbottomConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
  }
}
