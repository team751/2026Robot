package frc.robot.util;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.Idle;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.lib.PS5Controller;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.SpitCommand;
import frc.robot.commands.RetractCommand;
import frc.robot.commands.ExtendCommand;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.drive.SwerveConstants;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.simulation.MapSimSwerveTelemetry;

//TODO: Clean up the controller bindings for this
public class ControlBoard {
	private static ControlBoard instance;

	/* Controllers */
	private PS5Controller driver = null;
	private PS5Controller operator = null;
	private SwerveSubsystem drive = SwerveSubsystem.getInstance();
	private ShooterSubsystem shooter = ShooterSubsystem.getInstance();
	private ClimberSubsystem climber = ClimberSubsystem.getInstance();
	private boolean preciseControl = false;
	private boolean autoAim = false;
	private boolean axisAlign = false;

	private PIDController autoAimController = new PIDController(0.4, 0.0, 0.01);
	private ProfiledPIDController axisAlignController = new ProfiledPIDController(1.5, 0, 0.4, new Constraints(1, 1));

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

	/*Commands */
	private final IntakeCommand intakeCommand = new IntakeCommand();
	private final SpitCommand spitCommand = new SpitCommand();
	private final RetractCommand retractCommand = new RetractCommand();
	private final ExtendCommand extendCommand = new ExtendCommand();
	private final Idle IdleCommand = new Idle();

	private final SwerveRequest.FieldCentric driveRequest =
			new SwerveRequest.FieldCentric()
					.withDeadband(SwerveConstants.maxSpeed * 0.05)
					.withRotationalDeadband(SwerveConstants.maxAngularSpeed * 0.1)
					.withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage)
					.withSteerRequestType(SwerveModule.SteerRequestType.Position)
					.withDesaturateWheelSpeeds(true)
					.withForwardPerspective(SwerveRequest.ForwardPerspectiveValue.OperatorPerspective);

	private ControlBoard() {
		DriverStation.silenceJoystickConnectionWarning(true);
		autoAimController.enableContinuousInput(-180, 180);
		tryInit();
	}

	public void tryInit() {
		if (driver == null) {
			driver = new PS5Controller(ControllerPreset.DRIVER.port());
			configureBindings(ControllerPreset.DRIVER, driver);

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

	private static double getAxisAlignAngle(double currentDegrees) {
		currentDegrees = currentDegrees % 360;
		if (currentDegrees > 180) currentDegrees -= 360;
		if (currentDegrees < -180) currentDegrees += 360;
		return Math.abs(currentDegrees) <= 90 ? 0.0 : 180.0;
	}

	private void configureDriverBindings(PS5Controller controller) {
		/* Shooter */
		controller.triangleButton.whileTrue(
			new StartEndCommand(() -> shooter.newShooterSpeed(1, 1), () -> shooter.requestIdle())
		);

		controller.dLeft.whileTrue(
			new StartEndCommand(() -> shooter.newShooterSpeed(2, 2), () -> shooter.requestIdle())
		);

		controller.dUp.whileTrue(
			new StartEndCommand(() -> shooter.newShooterSpeed(5, 5), () -> shooter.requestIdle())
		);

		controller.dRight.whileTrue(
			new StartEndCommand(() -> shooter.newShooterSpeed(7, 7), () -> shooter.requestIdle())
		);

		controller.leftBumper.whileTrue(
			new StartEndCommand(() -> shooter.newShooterSpeed(2, 0), () -> shooter.requestIdle())
		);

		controller.rightTrigger.whileTrue(
			new StartEndCommand(() -> shooter.newShooterSpeed(0, 2), () -> shooter.requestIdle())
		);

		/* Climber */
		controller.leftTrigger.whileTrue(
			new StartEndCommand(() -> climber.spinSlow(1), () -> climber.stopMotors())
		);

		controller.crossButton.whileTrue(
			new StartEndCommand(() -> climber.spinSlow(-1), () -> climber.stopMotors())
		);

		/* Swerve Drive */
		controller.rightBumper.whileTrue(
				new StartEndCommand(() -> preciseControl = true, () -> preciseControl = false)
						.withName("Precise Control Toggle")); // Fight me owen
		/*Intake */
		controller.leftTrigger.whileTrue(
			intakeCommand);
		controller.leftTrigger.whileTrue(
			extendCommand);	
		controller.rightTrigger.whileTrue(
			retractCommand);
		controller.touchpadButton.whileTrue(
			spitCommand);					
	}


	private void configureOperatorBindings(PS5Controller controller) {
		controller.leftJoystickButton.onTrue(
			new InstantCommand(() -> drive.setRobotRotationByAlliance()));

		controller.circleButton.whileTrue(
			new InstantCommand(() -> climber.stopMotors()));

		controller.squareButton.whileTrue(
				new StartEndCommand(() -> autoAim = true, () -> autoAim = false)
						.withName("Auto Aim Toggle"));

		controller.rightJoystickButton.whileTrue(
				new StartEndCommand(() -> axisAlign = true, () -> axisAlign = false)
						.withName("Axis Align Toggle"));
	}

	public SwerveRequest getDriverRequest() {
		if (driver == null) return null;

		double scale = preciseControl ? 0.25 : 1.0;
		double rotScale = preciseControl ? 0.50 : 1.0;

		double rawStickRot = driver.rightHorizontalJoystick.getAsDouble();
		double rot = rotScale * SwerveConstants.maxAngularSpeed * (Math.copySign(rawStickRot * rawStickRot, rawStickRot));

		if (autoAim) {
			Pose2d robotPose = drive.getPose();
			Pose2d hubPose = FieldConstants.getAllianceHub();
			double angleDiff = Math.toDegrees(Math.atan2(hubPose.getY() - robotPose.getY(), hubPose.getX() - robotPose.getX()));
			SmartDashboard.putNumber("target offness", angleDiff - robotPose.getRotation().getDegrees());
			rot = autoAimController.calculate(robotPose.getRotation().getDegrees(), angleDiff);
			SmartDashboard.putNumber("pid value", rot);
		}

		double x = driver.leftVerticalJoystick.getAsDouble();
		double y;

		if (axisAlign) {
			Pose2d robotPose = drive.getPose();
			Pose2d nearestTrench = FieldConstants.getNearestTrench(robotPose);
			y = -axisAlignController.calculate(robotPose.getY(), nearestTrench.getY());
			rot = autoAimController.calculate(robotPose.getRotation().getDegrees(), getAxisAlignAngle(robotPose.getRotation().getDegrees()));
		} else {
			y = driver.leftHorizontalJoystick.getAsDouble();
		}

		return driveRequest
				.withVelocityX(0.6 * SwerveConstants.maxSpeed * x * scale)
				.withVelocityY(0.6 * SwerveConstants.maxSpeed * y * scale)
				.withRotationalRate(rot);
	}
}
