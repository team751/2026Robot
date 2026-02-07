package frc.robot.util;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.PS5Controller;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.simulation.MapSimSwerveTelemetry;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;
import frc.robot.subsystems.simulation.MapleSimSwerveDrivetrain;

public class ControlBoard {
	private static ControlBoard instance;

	/* Controllers */
	private PS5Controller driver = null;
	private PS5Controller operator = null;

	private boolean preciseControl = false;
	private double lastShotTime = 0.0; // Track last shot time for cooldown

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

		controller.rightTrigger.whileTrue(
				Commands.run(() -> {
					// Check cooldown - must wait 0.25 seconds between shots
					double currentTime = Timer.getFPGATimestamp();
					if (currentTime - lastShotTime < 0.3) {
						return; // Still in cooldown period
					}

					// Attempt to obtain a game piece from the intake
					// Exit early if no balls available - prevents spawning projectiles when empty
					if (!SwerveSubsystem.simDrivetrain.mapleSimIntake.obtainGamePieceFromIntake()) {
						return;
					}

					// Only create and launch projectile if we successfully obtained a ball
					RebuiltFuelOnFly fuelOnFly = new RebuiltFuelOnFly(
							// Specify the position of the chassis when the fuel is launched
							SwerveSubsystem.simDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose().getTranslation(),
							// Specify the translation of the shooter from the robot center (in the shooter's reference frame)
							new Translation2d(0.5, 0.2),
							// Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
							SwerveSubsystem.simDrivetrain.mapleSimDrive.getDriveTrainSimulatedChassisSpeedsRobotRelative(),
							// The shooter facing direction is the same as the robot's facing direction
							SwerveSubsystem.simDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose().getRotation(),
							// Initial height of the flying fuel
							Meters.of(1),
							// The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
							MetersPerSecond.of(5),
							// The angle at which the fuel is launched
							Radians.of(1.0472));

					SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);

					// Update last shot time for cooldown tracking
					lastShotTime = currentTime;
				}));		
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