package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ExtenderSubsystem extends SubsystemBase {
  private static ExtenderSubsystem instance;

  /* Motors */
  private final TalonFX extenderMotor =
      IntakeConstants.extenderMotorConfig.createDevice(TalonFX::new);

  /* Control Signals */
  private final VoltageOut extenderControl = new VoltageOut(0);

  /* Limit Switches */
  DigitalInput RightLimit = new DigitalInput(3);

  DigitalInput LeftLimit = new DigitalInput(2);

  /* State Machine Logic */
  private enum ExtenderState {
    IDLE,
    EXTENDING,
    RETRACTING,
  }

  private ExtenderState state = ExtenderState.IDLE;

  private boolean requestedIdle = false;
  private boolean requestedExtending = false;
  private boolean requestedRetracting = false;

  public static ExtenderSubsystem getInstance() {
    if (instance == null) instance = new ExtenderSubsystem();
    return instance;
  }

  private ExtenderSubsystem() {
    setExtenderMotor(0);
  }

  @Override
  public void periodic() {
    ExtenderState nextState = state;
    if (requestedIdle) nextState = ExtenderState.IDLE;
    else if (requestedExtending) nextState = ExtenderState.EXTENDING;
    else if (requestedRetracting) nextState = ExtenderState.RETRACTING;

    if (nextState != state) {
      state = nextState;
      unsetAllRequests();

      switch (state) {
        case IDLE -> setExtenderMotor(0);
        case EXTENDING -> setExtenderMotor(IntakeConstants.extenderSpeed);
        case RETRACTING -> setExtenderMotor(IntakeConstants.retractorSpeed);
      }
    }

    if (state == ExtenderState.EXTENDING && (LeftLimit.get() || RightLimit.get())) {
      setExtenderMotor(0.5);
    }
    if (state == ExtenderState.RETRACTING && (LeftLimit.get() || RightLimit.get())) {
      setExtenderMotor(0.5);
    }
    if (state == ExtenderState.RETRACTING && (RightLimit.get() && LeftLimit.get())) {
      requestIdle();
      setExtenderMotor(0);
    }
    if (state == ExtenderState.EXTENDING && (RightLimit.get() && LeftLimit.get())) {
      requestIdle();
      setExtenderMotor(0);
    }
    SmartDashboard.putNumber(
        "Extender/Extender Speed", extenderMotor.getVelocity().getValueAsDouble());
  }

  /**
   * Set the extender motor to a given speed
   *
   * @param voltage in volts
   */
  private void setExtenderMotor(double voltage) {
    extenderMotor.setControl(extenderControl.withOutput(voltage));
  }

  public void requestExtending() {
    requestedRetracting = false;
    requestedIdle = false;
    requestedExtending = true;
  }

  public void requestRetracting() {
    requestedExtending = false;
    requestedIdle = false;
    requestedRetracting = true;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestedIdle = true;
  }

  private void unsetAllRequests() {
    requestedIdle = false;
    requestedExtending = false;
    requestedRetracting = false;
  }
}
