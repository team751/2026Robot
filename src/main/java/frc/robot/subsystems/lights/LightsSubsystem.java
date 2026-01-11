package frc.robot.subsystems.lights;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightsSubsystem extends SubsystemBase {
	private static LightsSubsystem instance = null;
	private static final boolean doFadePercent = false;

	private final CANdle candle = LightsConstants.CANdleConfig.createDevice(CANdle::new);

	private boolean fading = false;
	private double fadeStartTime = 0.0;
	private double fadePercent = 0.0;
	private RGBWColor fadeStartColor = Colors.OFF;
	private RGBWColor fadeEndColor = Colors.OFF;

	private final Timer blinkTimer = new Timer();
	private boolean blinking = false;
	private boolean blinkOff = false;

	private RGBWColor currentColor = new RGBWColor();
	private final SolidColor solidColorControl =
			new SolidColor(0, LightsConstants.numLEDs).withColor(currentColor);

	public static final class Colors {
		public static final RGBWColor OFF = new RGBWColor(0, 0, 0);
		public static final RGBWColor WHITE = new RGBWColor(255, 255, 255);
		public static final RGBWColor RED = new RGBWColor(255, 0, 0);
		public static final RGBWColor YELLOW = new RGBWColor(255, 255, 0);
		public static final RGBWColor ORANGE = new RGBWColor(255, 165, 0);
		public static final RGBWColor GREEN = new RGBWColor(0, 255, 0);
		public static final RGBWColor CYAN = new RGBWColor(0, 255, 255);
		public static final RGBWColor BLUE = new RGBWColor(0, 0, 255);
		public static final RGBWColor PURPLE = new RGBWColor(132, 0, 255);
		public static final RGBWColor MAGENTA = new RGBWColor(255, 0, 255);
		public static final RGBWColor PINK = new RGBWColor(255, 192, 203);

		public static final RGBWColor TEAM_751 = new RGBWColor(48, 131, 255);
		public static final RGBWColor PICTION_BLUE = new RGBWColor(0, 171, 231);
		public static final RGBWColor PERSIAN_BLUE = new RGBWColor(37, 65, 178);
		public static final RGBWColor AQUAMARINE = new RGBWColor(166, 244, 220);
		public static final RGBWColor MAJORELLE_BLUE = new RGBWColor(114, 76, 249);
		public static final RGBWColor ULTRAVIOLET = new RGBWColor(86, 69, 146);

		public static final RGBWColor[] allColors =
				new RGBWColor[] {
					Colors.WHITE,
					Colors.RED,
					Colors.YELLOW,
					Colors.ORANGE,
					Colors.GREEN,
					Colors.CYAN,
					Colors.BLUE,
					Colors.PURPLE,
					Colors.MAGENTA,
					Colors.PINK,
					Colors.AQUAMARINE
				};
	}

	public static LightsSubsystem getInstance() {
		if (instance == null) instance = new LightsSubsystem();
		return instance;
	}

	private LightsSubsystem() {
		blinkTimer.start();
		requestColor(Colors.RED, true);
	}

	public void requestColor(RGBWColor color, boolean blink) {
		blinking = blink;
		blinkTimer.reset();
		fadeBetweenColors(color);
	}

	public void requestColor(RGBWColor color) {
		requestColor(color, blinking);
	}

	private void fadeBetweenColors(RGBWColor end) {
		fadeStartColor = new RGBWColor(currentColor.Red, currentColor.Green, currentColor.Blue);
		fadeEndColor = end;
		fadeStartTime = Timer.getFPGATimestamp();
		fading = true;

		candle.setControl(solidColorControl.withColor(currentColor));
	}

	public void requestRainbow() {
		//        candle.setControl(new RainbowAnimation(0, LightsConstants.numLEDs)
		//                .withBrightness(1)
		//                .withDirection(AnimationDirectionValue.Forward)
		//        );
		fading = false;
	}

	public void requestBlinking(boolean blink) {
		blinking = blink;
		candle.setControl(solidColorControl.withColor(currentColor)); // full brightness
		blinkOff = false; // reset blink state
		blinkTimer.reset();
	}

	public void requestToggleBlinking() {
		requestBlinking(!blinking);
	}

	@Override
	public void periodic() {
		if (fading) updateFade();
		if (blinking) updateBlink();
	}

	private void updateFade() {
		double elapsed = Timer.getFPGATimestamp() - fadeStartTime;
		double fraction =
				doFadePercent ? MathUtil.clamp(fadePercent, 0, 1) : elapsed / LightsConstants.fadeDuration;
		fraction = Math.min(fraction, 1.0);

		int red = (int) MathUtil.interpolate(fadeStartColor.Red, fadeEndColor.Red, fraction);
		int green = (int) MathUtil.interpolate(fadeStartColor.Green, fadeEndColor.Green, fraction);
		int blue = (int) MathUtil.interpolate(fadeStartColor.Blue, fadeEndColor.Blue, fraction);
		currentColor = new RGBWColor(red, green, blue);

		candle.setControl(
				solidColorControl.withColor(currentColor.scaleBrightness(blinkOff ? 0.125 : 1)));
		if (fraction >= 1.0) fading = false;
	}

	private void updateBlink() {
		if (blinkTimer.hasElapsed(LightsConstants.blinkInterval)) {
			blinkOff = !blinkOff;
			blinkTimer.reset();

			candle.setControl(
					solidColorControl.withColor(currentColor.scaleBrightness(blinkOff ? 0.125 : 1)));
		}
	}

	public void requestAllianceColor() {
		requestBlinking(false);
		requestColor(
				DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
								== DriverStation.Alliance.Red
						? Colors.RED
						: Colors.BLUE);
		// if (Climber.getInstance().isClimbing()) requestRainbow();
	}
}
