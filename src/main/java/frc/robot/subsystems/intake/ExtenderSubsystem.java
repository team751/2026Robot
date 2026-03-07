package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
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

  private final PIDController extenderPID = new PIDController(0.05, 0.0, 0.0);//, new Constraints(2.0, 1.0));

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
        case EXTENDING -> controlMotor(IntakeConstants.extenderLength + 5);
        case RETRACTING -> controlMotor(-5);
        case EXTENDED, RETRACTED -> setExtenderMotor(0);
      }
    }

    if (frontRightLimit.get() && frontLeftLimit.get()){
      estimatedExtension = 0.0;
    }else if (backRightLimit.get() && backLeftLimit.get()) {
      estimatedExtension = IntakeConstants.extenderLength;
    }{
      estimatedExtension += extenderMotor.getVelocity().getValueAsDouble() * IntakeConstants.extenderGearRatio * 0.02; // 0.02 is the loop time in seconds
    }

    if (state == ExtenderState.EXTENDING && (backLeftLimit.get() || backRightLimit.get())) {
      setExtenderMotor(IntakeConstants.extenderSpeed*0.25);
    }else if (state == ExtenderState.RETRACTING && (frontLeftLimit.get() || frontRightLimit.get())) {
      setExtenderMotor(IntakeConstants.retractorSpeed*0.25);
    }


    if (state == ExtenderState.EXTENDING && (backRightLimit.get() && backLeftLimit.get())) {
      state = ExtenderState.EXTENDED;
    } else if (state == ExtenderState.RETRACTING && (frontRightLimit.get() && frontLeftLimit.get())) {
      state = ExtenderState.RETRACTED;
    }

    SmartDashboard.putNumber(
        "Extender/Extender Speed", extenderMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber(
        "Extender/Extension", estimatedExtension);
  }

  /**
   * Set the extender motor to a given speed
   *
   * @param voltage in volts
   */
  private void setExtenderMotor(double voltage) {
    extenderMotor.setControl(extenderControl.withOutput(voltage));
  }

  private void controlMotor(double target) {
    double pidOutput = extenderPID.calculate(estimatedExtension, target);
    SmartDashboard.putNumber("Extender/PIDThing", pidOutput);
    setExtenderMotor(pidOutput);
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
