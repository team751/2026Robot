package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
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

  private final PIDController extenderPID = new PIDController(0.25, 0.0, 0.0);//, new Constraints(20.0, 10.0));
  //private final ProfiledPIDController extenderPID = new ProfiledPIDController(0.25, 0.0, 0.0, new Constraints(20.0, 10.0));

  DigitalInput frontLeftLimit = new DigitalInput(IntakeConstants.FrontLeftLimitID);
  DigitalInput backLeftLimit = new DigitalInput(IntakeConstants.BackLeftLimitID);
  DigitalInput frontRightLimit = new DigitalInput(IntakeConstants.FrontRightLimitID);
  DigitalInput backRightLimit = new DigitalInput(IntakeConstants.BackRightLimitID);

  private double m_lastTimestamp = Timer.getFPGATimestamp();
  private double deltaTime = 0.02;

  /* State Machine Logic */
  private enum ExtenderState {
    EXTENDED,
    RETRACTED,
    EXTENDING,
    RETRACTING
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
    double now = Timer.getFPGATimestamp();
    deltaTime = now - m_lastTimestamp;
    m_lastTimestamp = now;
    double motorOutput = 0;
    switch (state) {
      case EXTENDING -> motorOutput = Math.max(-3.0, Math.min(3.0, calculateMotorControl(IntakeConstants.extenderLength + 5)));
      case RETRACTING -> motorOutput = Math.max(-3.0, Math.min(3.0, calculateMotorControl(-5)));        
      case EXTENDED, RETRACTED -> motorOutput = 0;
    }

    setExtenderMotor(motorOutput);

    calculateExtension();

    if (state == ExtenderState.EXTENDING && isFullyExtended()) {
      state = ExtenderState.EXTENDED;
    } else if (state == ExtenderState.RETRACTING && isFullyRetracted()) {
      state = ExtenderState.RETRACTED;
    }

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

  private double calculateMotorControl(double target) {
    double pidOutput = extenderPID.calculate(estimatedExtension, target);
    SmartDashboard.putNumber("Extender/PIDThing", pidOutput);
    return pidOutput;
  }

  private void calculateExtension() {
    if (isFullyExtended()) {
      estimatedExtension = IntakeConstants.extenderLength;
    }else if (backRightLimit.get() && backLeftLimit.get()) {
      estimatedExtension = IntakeConstants.extenderLength;
    }else if (isFullyRetracted()) {
      estimatedExtension = 0.0;
    }else{
      estimatedExtension += extenderMotor.getVelocity().getValueAsDouble() * IntakeConstants.extenderGearRatio * 0.02; // 0.02 is the loop time in seconds
      estimatedExtension += extenderMotor.getVelocity().getValueAsDouble() * IntakeConstants.extenderGearRatio * deltaTime;
    }
  }

  private boolean atPosition(double target) {
    return Math.abs(estimatedExtension - target) < 1.0; // 1 cm tolerance
  }

  private boolean isFullyExtended() {
    return backLeftLimit.get() || backRightLimit.get();
  }

  private boolean isFullyRetracted() {
    return frontLeftLimit.get() || frontRightLimit.get();
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