package frc.robot.util;

import edu.wpi.first.hal.HAL;

public class Constants {
	// TODO: Swerve Motor ID's
	public static int frontLeftDriveID = 10;
	public static int frontLeftSteerID = 11;
	public static int frontLeftEncoderID = 12;

	public static int frontRightDriveID = 20;
	public static int frontRightSteerID = 21;
	public static int frontRightEncoderID = 22;

	public static int backLeftDriveID = 40;
	public static int backLeftSteerID = 41;
	public static int backLeftEncoderID = 42;

	public static int backRightDriveID = 30;
	public static int backRightSteerID = 31;
	public static int backRightEncoderID = 32;


	// TODO: Shooter Motor ID's
	public static int flywheelMotorID = 50;
	public static int backMotorID = 51;



	// TODO: Climber Motor ID's
	public static int leftClimbermMotorID = -1;
	public static int rightClimberMotorID = -1;



	// /** Use {@link frc.robot.Robot#drivebus} instead */
	// @SuppressWarnings("DeprecatedIsStillUsed")
	// @Deprecated
	// public static final String drivebus = "Drivebus";

	public static final boolean disableHAL = !HAL.initialize(500, 0);
}
