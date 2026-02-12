package frc.robot.util;

import edu.wpi.first.hal.HAL;

public class Constants {
	/** Use {@link frc.robot.Robot#drivebus} instead */
	@SuppressWarnings("DeprecatedIsStillUsed")
	@Deprecated
	public static final String drivebus = "Drivebus";

	public static final boolean disableHAL = !HAL.initialize(500, 0);
}
