package frc.robot.util;

import edu.wpi.first.hal.HAL;

public class Constants {
  // Swerve Motor ID's
  // Front Left
  public static final int frontLeftDriveID = 10;
  public static final int frontLeftSteerID = 11;
  public static final int frontLeftEncoderID = 12;

  // Front Right
  public static final int frontRightDriveID = 20;
  public static final int frontRightSteerID = 21;
  public static final int frontRightEncoderID = 22;

  // Back Left
  public static final int backLeftDriveID = 40;
  public static final int backLeftSteerID = 41;
  public static final int backLeftEncoderID = 42;

  // Back Right
  public static final int backRightDriveID = 30;
  public static final int backRightSteerID = 31;
  public static final int backRightEncoderID = 32;

  // Shooter Motor ID's
  public static final int flywheelMotorID = -1;
  public static final int backMotorID = -1;

  // Climber Motor ID's
  public static final int leftClimberMotorID = -1;
  public static final int rightClimberMotorID = -1;

  public static final int leftServoPort = -1;
  public static final int rightServoPort = -1;

  // Intake Motor ID's
  public static final int extenderMotorID = -1;
  public static final int intakeMotorID = -1;

  // /** Use {@link frc.robot.Robot#drivebus} instead */
  // @SuppressWarnings("DeprecatedIsStillUsed")
  // @Deprecated
  // public static final String drivebus = "Drivebus";

  public static final boolean disableHAL = !HAL.initialize(500, 0);
}
