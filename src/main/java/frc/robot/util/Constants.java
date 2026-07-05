package frc.robot.util;

import edu.wpi.first.hal.HAL;

/* Motor ID System:
 * Swerve motors are ID'd 0-2, 10-40 (10, 11, 12 or 30,31,32), starting from the Front Left and going clockwise around
 *
 * Every other motor is ID'd based on how close it is to each swerve module (if its in the middle, then based on the very back of the motor)
 * and how high it is on the robot (higher on the robot = lower number)
 *
 * For example, the main intake motor is on the front right of the robot, so it is closest to the front right of the robot. Despite being one of the
 * lowest motors on the robot, it is ID 23 because it is the only motor on the front right portion of the robot.
 *
 * The right transfer motor is ID 35 because it is on the back right of the robot. But there are two other motors (shooter motors) that are higher
 * than it, so it has a higher ID than those ones.
 */

public class Constants {
  public static class IDConstants {
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
  public static final int flywheelMotorID = 33;
  public static final int followMotorID = 43;

  public static final int shooterTransferMotorID = 34;

  // Climber Motor ID's
  // public static final int leftClimberMotorID = 46;
  // public static final int rightClimberMotorID = 36;

  // public static final int leftServoPort = -1;
  // public static final int rightServoPort = -1;

  // Intake Motor ID's
  public static final int intakeMotorID = 23;
  public static final int extenderMotorID = 13;

  // Transfer Motor ID's
  /**
   * This is the motor that controls the top of intake. Naming it the right motor because it is
   * physically located on the same plane as the other intake motor but on the right
   */
  public static final int rightTransferMotorID = 35;

  /**
   * This is the motor that controls the bottom of intake. Naming it the left motor because it is
   * physically located on the same plane as the other intake motor but on the left.
   */
  public static final int leftTransferMotorID = 45;

  // MegaTag switching is now dynamic — see LimelightConstants.MT_SWITCH_DISTANCE_METERS

  // /** Use {@link frc.robot.Robot#drivebus} instead */
  // @SuppressWarnings("DeprecatedIsStillUsed")
  // @Deprecated
  // public static final String drivebus = "Drivebus";

  public static final boolean disableHAL = !HAL.initialize(500, 0);
  }
}
