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
  DigitalInput frontLeftLimit = new DigitalInput(IntakeConstants.FrontLeftLimitID);
  DigitalInput backLeftLimit = new DigitalInput(IntakeConstants.BackLeftLimitID);
  DigitalInput frontRightLimit = new DigitalInput(IntakeConstants.FrontRightLimitID);
  DigitalInput backRightLimit = new DigitalInput(IntakeConstants.BackRightLimitID);

  /* State Machine Logic */
  private enum ExtenderState {
    IDLE,
    EXTENDING,
    RETRACTING
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

    if ((frontLeftLimit.get() || frontRightLimit.get()) && state == ExtenderState.RETRACTING) {
      requestIdle();
    }

    if ((backLeftLimit.get() || backRightLimit.get()) && state == ExtenderState.EXTENDING) {
      requestIdle();
    }

    SmartDashboard.putNumber(
        "Extender/Extender Speed", extenderMotor.getVelocity().getValueAsDouble());

    SmartDashboard.putString("Extender/State", state.toString());

    SmartDashboard.putBoolean("Extender/Front Left Limit", frontLeftLimit.get());
    SmartDashboard.putBoolean("Extender/Back Left Limit", backLeftLimit.get());
    SmartDashboard.putBoolean("Extender/Front Right Limit", frontRightLimit.get());
    SmartDashboard.putBoolean("Extender/Back Right Limit", backRightLimit.get());
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
    if (!backLeftLimit.get() && !backRightLimit.get()) {
      unsetAllRequests();
      requestedExtending = true;
    }
  }

  public void requestRetracting() {
    if (!frontLeftLimit.get() && !frontRightLimit.get()) {
      unsetAllRequests();
      requestedRetracting = true;
    }
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

  /** Returns true when the extender is fully extended (back limit switches triggered). */
  public boolean isAtExtendLimit() {
    return backLeftLimit.get() || backRightLimit.get();
  }

  /** Returns true when the extender is fully retracted (front limit switches triggered). */
  public boolean isAtRetractLimit() {
    return frontLeftLimit.get() || frontRightLimit.get();
  }
}
