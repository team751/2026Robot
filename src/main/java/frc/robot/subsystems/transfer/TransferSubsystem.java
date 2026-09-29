package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class TransferSubsystem extends SubsystemBase {
  private static TransferSubsystem instance;

  /* Motors */
  private final TalonFX topMotor = TransferConstants.transfertopconfig.createDevice(TalonFX::new);
  private final TalonFX bottomMotor =
      TransferConstants.transferbottomconfig.createDevice(TalonFX::new);

  /* Control Signals */
  private final VoltageOut topControl = new VoltageOut(0);
  private final VoltageOut bottomControl = new VoltageOut(0);

  /* State Machine Logic */
  public enum TransferState {
    IDLE,
    TRANSFER,
    REVERSE,
  }

  private TransferState state = TransferState.IDLE;

  public static TransferSubsystem getInstance() {
    if (instance == null) instance = new TransferSubsystem();
    return instance;
  }

  private TransferSubsystem() {
    setMotors(0, 0);
  }

  @Override
  public void periodic() {

    switch (state) {
      case IDLE -> setMotors(0, 0);
      case TRANSFER -> setMotors(
          TransferConstants.transfertopspeed, TransferConstants.transferbottomspeed);
      case REVERSE -> setMotors(
          -TransferConstants.transfertopspeed, -TransferConstants.transferbottomspeed);
    }

    Logger.recordOutput("Transfer/State", state);
    Logger.recordOutput("Transfer/TopVelocity", topMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput("Transfer/BottomVelocity", bottomMotor.getVelocity().getValueAsDouble());
    Logger.recordOutput(
        "Transfer/TopCurrent", topMotor.getStatorCurrent().getValueAsDouble());
    Logger.recordOutput(
        "Transfer/BottomCurrent", bottomMotor.getStatorCurrent().getValueAsDouble());
  }

  private void setMotors(double topVoltage, double bottomVoltage) {
    topMotor.setControl(topControl.withOutput(topVoltage));
    bottomMotor.setControl(bottomControl.withOutput(bottomVoltage));
  }

  public TransferState getState() {
    return state;
  }

  public void requestState(TransferState newState) {
    state = newState;
  }

  public void requestTransfer() {
    state = TransferState.TRANSFER;
  }

  public void requestReverse() {
    state = TransferState.REVERSE;
  }

  public void requestIdle() {
    state = TransferState.IDLE;
  }
}