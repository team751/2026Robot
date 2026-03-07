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

  private double estimatedExtension = 0.0;

  /* Control Signals */
  private final VoltageOut extenderControl = new VoltageOut(0);

  /* Limit Switches */
  DigitalInput frontRightLimit = new DigitalInput(3);

  DigitalInput frontLeftLimit = new DigitalInput(2);

  DigitalInput backRightLimit = new DigitalInput(4);

  DigitalInput backLeftLimit = new DigitalInput(5);

  /* State Machine Logic */
  private enum ExtenderState {
    EXTENDED,
	RETRACTED,
    EXTENDING,
    RETRACTING,
  }

  private ExtenderState state = ExtenderState.RETRACTED;
  private ExtenderState previousState = ExtenderState.RETRACTED;


  public static ExtenderSubsystem getInstance() {
    if (instance == null) instance = new ExtenderSubsystem();
    return instance;
  }

  @Override
  public void periodic() {
    if (previousState != state) {
      previousState = state;

      switch (state) {
        case EXTENDING -> setExtenderMotor(IntakeConstants.extenderSpeed);
        case RETRACTING -> setExtenderMotor(IntakeConstants.retractorSpeed);
		case EXTENDED -> setExtenderMotor(0);
		case RETRACTED -> setExtenderMotor(0);
      }
    }

	if (backRightLimit.get() && backLeftLimit.get()){
		estimatedExtension = 0.0
	}else{
		//estimatedExtension += extenderMotor.
	}

    if (state == ExtenderState.EXTENDING && (frontLeftLimit.get() || frontRightLimit.get())) {
      setExtenderMotor(IntakeConstants.extenderSpeed*0.25);
    }else if (state == ExtenderState.RETRACTING && (backLeftLimit.get() || backRightLimit.get())) {
      setExtenderMotor(IntakeConstants.retractorSpeed*0.25);
    }


    if (state == ExtenderState.EXTENDING && (frontRightLimit.get() && frontLeftLimit.get())) {
      state = ExtenderState.EXTENDED;
    } else if (state == ExtenderState.RETRACTING && (backRightLimit.get() && backLeftLimit.get())) {
      state = ExtenderState.RETRACTED;
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

  public void requestExtension() {
	state = ExtenderState.EXTENDING;
  }
  public void requestRetraction() {
	state = ExtenderState.RETRACTING;
  }
  public ExtenderState getState(){
	return state;
  }
}
