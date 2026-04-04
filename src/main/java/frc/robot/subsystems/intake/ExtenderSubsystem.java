package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
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

  private final PIDController extenderPID =
      new PIDController(0.2, 0.0, 0.0); // , new Constraints(20.0, 10.0));
  // private final ProfiledPIDController  bhextenderPID = new ProfiledPIDController(0.25, 0.0, 0.0,
  // new
  // Constraints(20.0, 10.0));

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
    JIGGLE_IN,
  }

  private ExtenderState state = ExtenderState.RETRACTED;

  public static ExtenderSubsystem getInstance() {
    if (instance == null) instance = new ExtenderSubsystem();
    return instance;
  }

  private ExtenderSubsystem() {
    setExtenderMotor(0);
  }

  @Override
  public void periodic() {
    double motorOutput = 0;
    switch (state) {
      case EXTENDING -> motorOutput =
          Math.max(-6.0, Math.min(6.0, calculateMotorControl(IntakeConstants.extenderLength + 7)));
      case RETRACTING -> motorOutput = Math.max(-5.0, Math.min(5.0, calculateMotorControl(-7)));
      case EXTENDED, RETRACTED -> motorOutput = 0;
      case JIGGLE_IN -> motorOutput = -1.5;
    }

    if ((state == ExtenderState.RETRACTING || state == ExtenderState.JIGGLE_IN)
        && !isExtended()
        && Math.abs(extenderMotor.getVelocity().getValueAsDouble()) < 0.05) {
      state = ExtenderState.EXTENDING;
    }

    setExtenderMotor(motorOutput);

    calculateExtension();

    if (state == ExtenderState.EXTENDING && isExtended()) {
      state = ExtenderState.EXTENDED;
    } else if ((state == ExtenderState.RETRACTING || state == ExtenderState.JIGGLE_IN)
        && isRetracted()) {
      state = ExtenderState.RETRACTED;
    }

    //SmartDashboard.putNumber("Extender/Extension", estimatedExtension);
  }

  /**
   * Set the extender motor to a given speed
   *
   * @param voltage in volts
   */
  private void setExtenderMotor(double voltage) {
    extenderMotor.setControl(extenderControl.withOutput(voltage));
  }

  private double calculateMotorControl(double target) {
    double pidOutput = extenderPID.calculate(estimatedExtension, target);
    //SmartDashboard.putNumber("Extender/PIDThing", pidOutput);
    return pidOutput;
  }

  private void calculateExtension() {
    if (isExtended()) {
      estimatedExtension = IntakeConstants.extenderLength;
    } else if (backRightLimit.get() && backLeftLimit.get()) {
      estimatedExtension = IntakeConstants.extenderLength;
    } else if (isRetracted()) {
      estimatedExtension = 0.0;
    } else {
      estimatedExtension +=
          extenderMotor.getVelocity().getValueAsDouble()
              * IntakeConstants.extenderGearRatio
              * 0.02; // 0.02 is the loop time in seconds
    }
  }

  public boolean isExtended() {
    return backLeftLimit.get() || backRightLimit.get();
  }

  public boolean isRetracted() {
    return frontLeftLimit.get() || frontRightLimit.get();
  }

  public void requestExtend() {
    state = ExtenderState.EXTENDING;
  }

  public void requestRetract() {
    state = ExtenderState.RETRACTING;
  }

  public void requestJiggleIn() {
    state = ExtenderState.JIGGLE_IN;
  }

  public ExtenderState getState() {
    return state;
  }
}
