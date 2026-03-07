package frc.robot.subsystems.climber;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  // TODO: figure out how to we want the climber to work. should the motors runtoposition? just run
  // until driver says so? what seems more efficient/convienent/helpful?

  private final TalonFX leftClimber = ClimberConstants.leftClimberConfig.createDevice(TalonFX::new);
  private final TalonFX rightClimber =
      ClimberConstants.rightClimberConfig.createDevice(TalonFX::new);
  private final Servo m_servo = new Servo(ClimberConstants.kServoPort);

  private final DigitalInput LeftLimit = new DigitalInput(8);
  private final DigitalInput RightLimit = new DigitalInput(9);

  private final PositionVoltage positionRequest = new PositionVoltage(0);

  private static ClimberSubsystem instance;

  // State used to run a non-blocking "spin until position" behavior
  private boolean spinning = false;
  private double spinTarget = 0.0;

  private boolean inverted = false;

  private double m_servoPosition = 0.0; // accumulated rotations
  private double m_servoSpeed = 0.0; // last commanded speed
  private double m_lastTimestamp; // initialized in constructor

  public static ClimberSubsystem getInstance() {
    if (instance == null) instance = new ClimberSubsystem();
    return instance;
  }

  private ClimberSubsystem() {
    m_servo.setBoundsMicroseconds(2400, 1520, 1500, 1480, 600);
    m_lastTimestamp = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    double now = Timer.getFPGATimestamp();
    m_servoPosition += m_servoSpeed * ClimberConstants.kServoMaxRPS * (now - m_lastTimestamp);
    m_lastTimestamp = now;

    SmartDashboard.putNumber("Left Motor Pos", leftClimber.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Right Motor Pos", rightClimber.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Servo Position (rotations)", m_servoPosition);

    SmartDashboard.putBoolean("Climb Left Limit", LeftLimit.get());
    SmartDashboard.putBoolean("Climb Right Limit", RightLimit.get());

    // Spin to target check
    if (spinning) {
      SmartDashboard.putNumber("SpinUntil/Target", spinTarget);

      // If its not inverted, check if the value of the motor is greater than or equal to the target
      // and then
      if (!inverted && leftClimber.getPosition().getValueAsDouble() >= spinTarget) {
        // stop the motors and set the spinning value to false
        stopSpinUntil();
        spinning = false;
        zeroClimber();
      }

      // If it is inverted, check if the value of the motor is less than or equal to the target and
      // then
      if (inverted && leftClimber.getPosition().getValueAsDouble() <= spinTarget) {
        // stop the motors and set the spinning value to false.
        stopSpinUntil();
        spinning = false;
        zeroClimber();
      }
    }
  }

  public void spinSlow(int direction) {
    leftClimber.setControl(new DutyCycleOut(direction * 0.1));
  }

  public void spinUntil(double value) {

    // Error and setup
    spinTarget = value - ClimberConstants.averageMotorError;
    spinning = true;

    if (spinning) {
      // Inversion check
      if (leftClimber.getPosition().getValueAsDouble() > spinTarget) {
        inverted = true;

        // Set the motors to spin
        leftClimber.setControl(new DutyCycleOut(-0.1));
        rightClimber.setControl(new StrictFollower(10));
      } else {
        inverted = false;

        // Set the motors to spin
        leftClimber.setControl(new DutyCycleOut(0.1));
        rightClimber.setControl(new StrictFollower(10));
      }
    }
  }

  public void stopSpinUntil() {
    spinning = false;

    // Get the average error
    double error = spinTarget - leftClimber.getPosition().getValueAsDouble();
    ClimberConstants.averageMotorError = (ClimberConstants.averageMotorError + error) / 3;

    stopMotors();
  }

  public void moveUp180() {

    double currentPosition = leftClimber.getPosition().getValueAsDouble();
    double targetPosition = currentPosition + 0.6;

    leftClimber.setControl(positionRequest.withPosition(targetPosition));
    rightClimber.setControl(positionRequest.withPosition(targetPosition));
  }

  public void moveDown180() {

    double currentPosition = leftClimber.getPosition().getValueAsDouble();
    double targetPosition = currentPosition - 0.6;

    leftClimber.setControl(positionRequest.withPosition(targetPosition));
    rightClimber.setControl(positionRequest.withPosition(targetPosition));
  }

  public void stopMotors() {
    leftClimber.stopMotor();
    rightClimber.stopMotor();
  }

  public void zeroClimber() {
    leftClimber.setPosition(0);
    rightClimber.setPosition(0);
  }

  public void setServoSpeed(double speed) {
    m_servoSpeed = speed;
    m_servo.setSpeed(speed);
  }

  public void stopServo() {
    m_servoSpeed = 0.0;
    m_servo.setSpeed(0.0);
  }

  public double getServoPosition() {
    return m_servoPosition;
  }

  public void resetServoPosition() {
    m_servoPosition = 0.0;
  }
}
