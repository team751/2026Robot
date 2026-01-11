package frc.robot.subsystems.lights;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.*;
import frc.lib.CTREConfig;
import frc.robot.Robot;

public class LightsConstants {
	private static final int CANdleOnboardLEDs = 8;

	public static final int numLEDs = 63 + CANdleOnboardLEDs;

	public static final double blinkInterval = 0.2;

	public static final double fadeDuration = 1.10;

	public static final CTREConfig<CANdle, CANdleConfiguration> CANdleConfig =
			new CTREConfig<>(CANdleConfiguration::new);

	static {
		CANdleConfig.withName("CANdle").withCanID(5).withBus(Robot.riobus);

		CANdleConfiguration config = CANdleConfig.config;
		config.CANdleFeatures.Enable5VRail = Enable5VRailValue.Enabled;
		config.CANdleFeatures.VBatOutputMode = VBatOutputModeValue.Off;
		config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;
		config.LED.BrightnessScalar = 1.0;
		config.LED.LossOfSignalBehavior = LossOfSignalBehaviorValue.KeepRunning;
		config.LED.StripType = StripTypeValue.GRB;
	}
}
