package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.FieldConstants;

public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem instance;

  // Motors
  private final TalonFX flywheelMotor =
      ShooterConstants.flywheelMotorConfig.createDevice(TalonFX::new);
  private final VelocityVoltage flywheelControl = new VelocityVoltage(0);

  private final TalonFX followMotor = ShooterConstants.followMotorConfig.createDevice(TalonFX::new);
  private final Follower followControl =
      new Follower(ShooterConstants.flywheelMotorConfig.canID, MotorAlignmentValue.Opposed);

  private final TalonFX shooterTransferMotor =
      ShooterConstants.transferMotorConfig.createDevice(TalonFX::new);
  // private final Follower shooterTransferControl = new
  // Follower(ShooterConstants.flywheelMotorConfig.canID, MotorAlignmentValue.Aligned);
  private final VoltageOut shooterTransferControl = new VoltageOut(0);

  /* State Machine Logic */
  private enum ShooterState {
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
    flywheelMotor.setControl(flywheelControl);
    followMotor.setControl(followControl);
    shooterTransferMotor.setControl(shooterTransferControl);

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
        case IDLE -> setShooterSpeed(0, 0);
        case SLOWSPIN -> setShooterMotor(
            ShooterConstants.flywheelSpeed * ShooterConstants.slowPercent);
        case SPINNING -> setShooterSpeed(calculateShooterSpeed(), ShooterConstants.transferVoltage);
      }
    }
  }

  /** Runs just the main flywheel motor */
  private void setShooterMotor(double flywheelVelocity) {
    flywheelMotor.setControl(flywheelControl.withVelocity(flywheelVelocity));
  }

  /** Runs just the transfer motor on shooter */
  // TODO: change to private, made public for testing stuff
  public void setTransferMotor(double transferVoltage) {
    shooterTransferMotor.setControl(shooterTransferControl.withOutput(transferVoltage));
  }

  /** Runs both the main shooter motor and transfer motor */
  private void setShooterSpeed(double flywheelVelocity, double transferVoltage) {
    flywheelMotor.setControl(flywheelControl.withVelocity(flywheelVelocity));
    shooterTransferMotor.setControl(shooterTransferControl.withOutput(transferVoltage));
  }

  public void increaseSpeed() {
    ShooterConstants.flywheelSpeed++;
  }

  public void decreaseSpeed() {
    ShooterConstants.flywheelSpeed--;
  }

  public void speed30() {
    ShooterConstants.flywheelSpeed = 30;
  }

  public void resetSpeed() {
    ShooterConstants.flywheelSpeed = 49;
  }

  private void unsetAllRequests() {
    requestedIdle = false;
    requestedShoot = false;
  }

  private double getRobotDistanceFromHub() {
    SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    Pose2d hubPose = FieldConstants.getAllianceHub();
    Pose2d robotPose = swerve.getPose();
    return 100 * Math.hypot(hubPose.getX() - robotPose.getX(), hubPose.getY() - robotPose.getY());
  }

  private boolean canShoot() {
    double distanceCM = getRobotDistanceFromHub();
    return distanceCM >= ShooterConstants.minShootingDistance
        && distanceCM <= ShooterConstants.maxShootingDistance;
  }

  private double calculateShooterSpeed() {
    // if (!canShoot()) {
    //   return ShooterConstants.flywheelSpeed;
    // }

    double distanceCM = 300; //getRobotDistanceFromHub();
    return (distanceCM - ShooterConstants.shooterDistanceCurveYIntercept)
        / ShooterConstants.shooterDistanceCurveSlope;
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
