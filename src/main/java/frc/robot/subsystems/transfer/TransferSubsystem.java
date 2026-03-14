package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: need to test this

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
  private enum TransferState {
    IDLE,
    TRANSFERRING,
    REVERSING,
  }

  private TransferState state = TransferState.IDLE;

  private boolean requestedIdle = false;
  private boolean requestedTransferring = false;
  private boolean requestedReversing = false;

  public static TransferSubsystem getInstance() {
    if (instance == null) instance = new TransferSubsystem();
    return instance;
  }

  private TransferSubsystem() {
    setMotors(0, 0);
  }

  @Override
  public void periodic() {
    TransferState nextState = state;
    if (requestedIdle) nextState = TransferState.IDLE;
    else if (requestedTransferring) nextState = TransferState.TRANSFERRING;
    else if (requestedReversing) nextState = TransferState.REVERSING;

    if (nextState != state) {
      state = nextState;
      unsetAllRequests();

      switch (state) {
        case IDLE -> setMotors(0, 0);
        case TRANSFERRING -> setMotors(
            TransferConstants.transfertopspeed, TransferConstants.transferbottomspeed);
        case REVERSING -> setMotors(
            -TransferConstants.transfertopspeed, -TransferConstants.transferbottomspeed);
      }
    }

    SmartDashboard.putString("Transfer/State", state.toString());
    SmartDashboard.putNumber("Transfer/Top Speed", topMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Transfer/Bottom Speed", bottomMotor.getVelocity().getValueAsDouble());
  }

  private void setMotors(double topVoltage, double bottomVoltage) {
    topMotor.setControl(topControl.withOutput(topVoltage));
    bottomMotor.setControl(bottomControl.withOutput(bottomVoltage));
  }

  public void requestTransferring() {
    requestedIdle = false;
    requestedReversing = false;
    requestedIdle = false;
    requestedTransferring = true;
  }

  public void requestReversing() {
    requestedIdle = false;
    requestedTransferring = false;
    requestedReversing = true;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestedIdle = true;
  }

  private void unsetAllRequests() {
    requestedIdle = false;
    requestedTransferring = false;
    requestedReversing = false;
  }
}
