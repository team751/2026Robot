package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem instance;

  // Motors
  private final TalonFX flywheelMotor =
      ShooterConstants.flywheelMotorConfig.createDevice(TalonFX::new);
  private final VoltageOut flywheelControl = new VoltageOut(0);

  private final TalonFX followMotor = ShooterConstants.followMotorConfig.createDevice(TalonFX::new);
  private final Follower followControl = new Follower(ShooterConstants.flywheelMotorConfig.canID, MotorAlignmentValue.Opposed);

  /* State Machine Logic */
  private enum ShooterState {
    // TODO: Do flywheel as closed loop and add follower mode for motors
    IDLE,
    SLOWSPIN,
    SPINNING
  }

  private ShooterState state = ShooterState.IDLE;

  private boolean requestedIdle = false;
  private boolean requestedSlow = false;
  private boolean requestedShoot = false;

  public static ShooterSubsystem getInstance() {
    if (instance == null) instance = new ShooterSubsystem();
    return instance;
  }

  private ShooterSubsystem() {
    followMotor.setControl(followControl);

    setShooterMotor(0);
  }

  @Override
  public void periodic() {
    ShooterState nextState = state;
    if (requestedIdle) nextState = ShooterState.IDLE;
    else if (requestedSlow) nextState = ShooterState.SLOWSPIN;
    else if (requestedShoot) nextState = ShooterState.SPINNING;

    if (nextState != state) {
      state = nextState;
      unsetAllRequests();

      switch (state) {
        case IDLE -> setShooterMotor(0);
        case SLOWSPIN -> setShooterMotor(0);
        case SPINNING -> setShooterMotor(ShooterConstants.flywheelSpeed);
      }
    }
  }

  private void setShooterMotor(double flywheelVoltage) {
    flywheelMotor.setControl(flywheelControl.withOutput(flywheelVoltage));
  }

  public void newSpeed(double flySpeed, double followSpeed) {
    ShooterConstants.flywheelSpeed = flySpeed;
    ShooterConstants.followSpeed = followSpeed;
    this.requestShoot();
  }

  private void unsetAllRequests() {
    requestedIdle = false;
    requestedShoot = false;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestedIdle = true;
  }

  public void requestShoot() {
    unsetAllRequests();
    requestedShoot = true;
  }
}
