package frc.robot.subsystems.transfer;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Runs the two transfer motors (top/bottom) that move a game piece from the intake toward the
 * shooter. Separate from — and not to be confused with — {@link
 * frc.robot.subsystems.shooter.ShooterSubsystem}'s own internal transfer motor, which is a distinct
 * mechanism right at the flywheel. {@link #requestState} lets callers like {@link
 * frc.robot.commands.SimpleUnstuck} save and restore an exact state rather than just idling.
 */
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
  }

  private void setMotors(double topVoltage, double bottomVoltage) {
    topMotor.setControl(topControl.withOutput(topVoltage));
    bottomMotor.setControl(bottomControl.withOutput(bottomVoltage));
    // SmartDashboard.putNumber("Transfer/Top Speed", topMotor.getVelocity().getValueAsDouble());
    // SmartDashboard.putNumber("Transfer/Bottom Speed",
    // bottomMotor.getVelocity().getValueAsDouble());
  }

  public TransferState getState() {
    return state;
  }

  public void requestState(TransferState newState) {
    state = newState;
    // SmartDashboard.putString("Transfer/State", state.toString());
  }

  public void requestTransfer() {
    state = TransferState.TRANSFER;
    // SmartDashboard.putString("Transfer/State", state.toString());
  }

  public void requestReverse() {
    state = TransferState.REVERSE;
    // SmartDashboard.putString("Transfer/State", state.toString());
  }

  public void requestIdle() {
    state = TransferState.IDLE;
    // SmartDashboard.putString("Transfer/State", state.toString());
  }
}
