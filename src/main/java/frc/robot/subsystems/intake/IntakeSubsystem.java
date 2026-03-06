package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  private static IntakeSubsystem instance;

  /* Motors */
  private final TalonFX intakeMotor = IntakeConstants.intakeMotorConfig.createDevice(TalonFX::new);

  /* Control Signals */
  private final VoltageOut intakeControl = new VoltageOut(0);

  /* State Machine Logic */
  private enum IntakeState {
    IDLE,
    INTAKING,
    SPITTING,
  }

  private IntakeState state = IntakeState.IDLE;

  private boolean requestedIdle = false;
  private boolean requestedIntaking = false;
  private boolean requestedSpitting = false;

  public static IntakeSubsystem getInstance() {
    if (instance == null) instance = new IntakeSubsystem();
    return instance;
  }

  private IntakeSubsystem() {
    setIntakeMotor(0);
  }

  @Override
  public void periodic() {
    IntakeState nextState = state;
    if (requestedIdle) nextState = IntakeState.IDLE;
    else if (requestedIntaking) nextState = IntakeState.INTAKING;
    else if (requestedSpitting) nextState = IntakeState.SPITTING;

    if (nextState != state) {
      state = nextState;
      unsetAllRequests();

      switch (state) {
        case IDLE -> setIntakeMotor(0);
        case INTAKING -> setIntakeMotor(IntakeConstants.intakeSpeed);
        case SPITTING -> setIntakeMotor(IntakeConstants.spitSpeed);
      }
    }
    SmartDashboard.putString("Intake/Intake State", state.toString());
    SmartDashboard.putNumber("Intake/Intake Speed", intakeMotor.getVelocity().getValueAsDouble());
  }

  /**
   * Set the intake motor to a given speed
   *
   * @param voltage in volts
   */
  private void setIntakeMotor(double voltage) {
    intakeMotor.setControl(intakeControl.withOutput(voltage));
  }

  public void requestIntaking() {
    requestedIdle = false;
    requestedSpitting = false;
    requestedIntaking = true;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestedIdle = true;
  }

  public void requestSpitting() {
    requestedIdle = false;
    requestedIntaking = false;
    requestedSpitting = true;
  }

  private void unsetAllRequests() {
    requestedIdle = false;
    requestedIntaking = false;
    requestedSpitting = false;
  }
}
