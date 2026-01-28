package frc.robot.util;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.lib.PS5Controller;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.simulation.MapSimSwerveTelemetry;

public class ControlBoard {
	private static ControlBoard instance;

	/* Controllers */
	private PS5Controller driver = null;
	private PS5Controller operator = null;

	private boolean preciseControl = false;

	private enum ControllerPreset {
		DRIVER(0),
		OPERATOR(1);

		private final int port;

		ControllerPreset(int port) {
			this.port = port;
		}

		public int port() {
			return port;
		}
	}

	/* Subsystems */
	private final Superstructure superstructure = Superstructure.getInstance();

	private final SwerveRequest.FieldCentric driveRequest =
			new SwerveRequest.FieldCentric()
					.withDeadband(SwerveConstants.maxSpeed * 0.05) // Add a 5% deadband
					.withRotationalDeadband(SwerveConstants.maxAngularSpeed * 0.1) // Add a 10% deadband
					.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
					.withSteerRequestType(SwerveModule.SteerRequestType.Position)
					.withDesaturateWheelSpeeds(true);

	private ControlBoard() {
		DriverStation.silenceJoystickConnectionWarning(true);

		tryInit();
	}

	public void tryInit() {
		if (driver == null) {
			driver = new PS5Controller(ControllerPreset.DRIVER.port());
			configureBindings(ControllerPreset.DRIVER, driver);

			SwerveSubsystem drive = SwerveSubsystem.getInstance();
			drive.setDefaultCommand(drive.applyRequest(this::getDriverRequest));
			if (Utils.isSimulation())
				drive.registerTelemetry(new MapSimSwerveTelemetry(SwerveConstants.maxSpeed)::telemeterize);
			System.out.println("Driver Initialized");
		}

		if (operator == null) {
			operator = new PS5Controller(ControllerPreset.OPERATOR.port());
			configureBindings(ControllerPreset.OPERATOR, operator);
			System.out.println("Operator Initialized");
		}
	}

	public void displayUI() {}

	public static ControlBoard getInstance() {
		if (instance == null) instance = new ControlBoard();
		return instance;
	}

	private void configureBindings(ControllerPreset preset, PS5Controller controller) {
		switch (preset) {
			case DRIVER -> configureDriverBindings(controller);
			case OPERATOR -> configureOperatorBindings(controller);
			default -> throw new IllegalStateException("Unexpected value: " + preset);
		}
	}

	private void configureDriverBindings(PS5Controller controller) {
		/* Precise Control */
		controller.rightBumper.whileTrue(
				new StartEndCommand(() -> preciseControl = true, () -> preciseControl = false)
						.withName("Precise Control Toggle")); // Fight me owen
	}

	private void configureOperatorBindings(PS5Controller controller) {}

	public SwerveRequest getDriverRequest() {
		if (driver == null) return null;

		double scale = preciseControl ? 0.5 : 1.0;
		double rotScale = preciseControl ? 0.50 : 1.0;

		double x = driver.leftVerticalJoystick.getAsDouble();
		double y = driver.leftHorizontalJoystick.getAsDouble();
		double rot = driver.rightHorizontalJoystick.getAsDouble();
		return driveRequest
				.withVelocityX(0.6 * SwerveConstants.maxSpeed * x * scale)
				.withVelocityY(0.6 * SwerveConstants.maxSpeed * y * scale)
				.withRotationalRate(
						0.8 * SwerveConstants.maxAngularSpeed * (Math.copySign(rot * rot, rot) * rotScale));
	}
}
