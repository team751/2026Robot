package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ExtenderSubsystem extends SubsystemBase {
  private static ExtenderSubsystem instance;

  /* Motors */
  private final TalonFX extenderMotor =
      IntakeConstants.extenderMotorConfig.createDevice(TalonFX::new);

  private double estimatedExtension = 0.0;

  /* Control Signals */
  private final VoltageOut extenderControl = new VoltageOut(0);

  private final ProfiledPIDController extenderPID = new ProfiledPIDController(1.0, 0.0, 0.1, new Constraints(1.0, 1.0));

  DigitalInput frontLeftLimit = new DigitalInput(IntakeConstants.FrontLeftLimitID);
  DigitalInput backLeftLimit = new DigitalInput(IntakeConstants.BackLeftLimitID);
  DigitalInput frontRightLimit = new DigitalInput(IntakeConstants.FrontRightLimitID);
  DigitalInput backRightLimit = new DigitalInput(IntakeConstants.BackRightLimitID);

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

  private ExtenderSubsystem() {
    setExtenderMotor(0);
  }


  @Override
  public void periodic() {
    if (previousState != state) {
      previousState = state;

      switch (state) {
        case EXTENDING -> setExtenderMotor(IntakeConstants.extenderSpeed);
        case RETRACTING -> setExtenderMotor(IntakeConstants.retractorSpeed);
        case EXTENDED, RETRACTED -> setExtenderMotor(0);
      }
    }
    // dw paigus
    controlMotor();

    if (backRightLimit.get() && backLeftLimit.get()){
      estimatedExtension = 0.0;
    }else{
      estimatedExtension += extenderMotor.getVelocity().getValueAsDouble() * IntakeConstants.extenderGearRatio * 0.02; // 0.02 is the loop time in seconds
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

      SmartDashboard.putBoolean("Front Right Limit", frontRightLimit.get());
      SmartDashboard.putBoolean("Back Right Limit", backRightLimit.get());
      SmartDashboard.putBoolean("Front Left Limit", frontLeftLimit.get());
      SmartDashboard.putBoolean("Back Left Limit", backLeftLimit.get());
  }

  /**
   * Set the extender motor to a given speed
   *
   * @param voltage in volts
   */
  private void setExtenderMotor(double voltage) {
    extenderMotor.setControl(extenderControl.withOutput(voltage));
  }

  private void controlMotor() {
    double pidOutput = extenderPID.calculate(estimatedExtension, state == ExtenderState.EXTENDING ? IntakeConstants.extenderLength : 0);
    SmartDashboard.putNumber("Extender/PIDThing", pidOutput);
    //setExtenderMotor(pidOutput);
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
