package frc.robot.subsystems.drive;

import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.generated.TunerConstants;
import frc.robot.subsystems.drive.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.simulation.MapleSimSwerveDrivetrain;
import java.util.function.Supplier;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.system.plant.DCMotor;

public class SwerveSubsystem extends TunerSwerveDrivetrain implements Subsystem {
	private static final double kSimLoopPeriod = 0.002; // 2 ms or 50hz
	private Notifier m_simNotifier = null;
	private double m_lastSimTime;

	/* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
	private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
	/* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
	private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
	/* Keep track if we've ever applied the operator perspective before or not */
	private boolean m_hasAppliedOperatorPerspective = false;

	/* Swerve requests to apply during SysId characterization */
	private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
			new SwerveRequest.SysIdSwerveTranslation();
	private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
			new SwerveRequest.SysIdSwerveSteerGains();
	private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
			new SwerveRequest.SysIdSwerveRotation();

	/* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
	private final SysIdRoutine m_sysIdRoutineTranslation =
			new SysIdRoutine(
					new SysIdRoutine.Config(
							null, // Use default ramp rate (1 V/s)
							Units.Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
							null, // Use default timeout (10 s)
							// Log state with SignalLogger class
							state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
					new SysIdRoutine.Mechanism(
							output -> setControl(m_translationCharacterization.withVolts(output)), null, this));

	/* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
	private final SysIdRoutine m_sysIdRoutineSteer =
			new SysIdRoutine(
					new SysIdRoutine.Config(
							null, // Use default ramp rate (1 V/s)
							Units.Volts.of(7), // Use dynamic voltage of 7 V
							null, // Use default timeout (10 s)
							// Log state with SignalLogger class
							state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
					new SysIdRoutine.Mechanism(
							volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

	/*
	 * SysId routine for characterizing rotation.
	 * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
	 * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
	 */
	private final SysIdRoutine m_sysIdRoutineRotation =
			new SysIdRoutine(
					new SysIdRoutine.Config(
							/* This is in radians per second², but SysId only supports "volts per second" */
							Units.Volts.of(Math.PI / 6).per(Units.Second),
							/* This is in radians per second, but SysId only supports "volts" */
							Units.Volts.of(Math.PI),
							null, // Use default timeout (10 s)
							// Log state with SignalLogger class
							state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
					new SysIdRoutine.Mechanism(
							output -> {
								/* output is actually radians per second, but SysId only supports "volts" */
								setControl(m_rotationCharacterization.withRotationalRate(output.in(Units.Volts)));
								/* also log the requested output for SysId */
								SignalLogger.writeDouble("Rotational_Rate", output.in(Units.Volts));
							},
							null,
							this));

	/* Autonomous Controllers */
	private final PIDController m_pathXController = new PIDController(1.5, 0.0, 0.0);
	private final PIDController m_pathYController = new PIDController(1.5, 0.0, 0.0);
	private final PIDController m_pathThetaController = new PIDController(10, 0.0, 0.0);

	private final SwerveRequest.ApplyFieldSpeeds m_pathApplyFieldSpeeds =
			new SwerveRequest.ApplyFieldSpeeds()
					.withSteerRequestType(SwerveModule.SteerRequestType.Position)
					.withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

	private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
			new SwerveRequest.ApplyRobotSpeeds()
					.withSteerRequestType(SwerveModule.SteerRequestType.Position)
					.withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

	/* The SysId routine to test */
	private final SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineSteer;

	private static SwerveSubsystem instance = null;

	public static SwerveSubsystem getInstance() {
		if (instance == null) {
			instance =
					new SwerveSubsystem(
							TunerConstants.DrivetrainConstants,
							TunerConstants.FrontLeft,
							TunerConstants.FrontRight,
							TunerConstants.BackLeft,
							TunerConstants.BackRight);
		}
		return instance;
	}

	/**
	 * Constructs a CTRE SwerveDrivetrain using the specified constants.
	 *
	 * <p>This constructs the underlying hardware devices, so users should not construct the devices
	 * themselves. If they need the devices, they can access them through getters in the classes.
	 *
	 * @param drivetrainConstants Drivetrai n-wide constants for the swerve drive
	 * @param modules Constants for each specific module
	 */
	public SwerveSubsystem(
			SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
		// super(drivetrainConstants, 250, VecBuilder.fill(0.1, 0.1, 0), VecBuilder.fill(0.1, 0.1,
		// 999999), MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));
		super(
				drivetrainConstants,
				MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules);
		PathFollowingController controller =
				new PPHolonomicDriveController(
						new PIDConstants(7.51, 0.0, 0.0), new PIDConstants(1, 0.0, 0.0));
		CommandScheduler.getInstance()
				.registerSubsystem(this); // Since it doesnt extend SubsystemBase ahhhhhh
		AutoBuilder.configure(
				this::getPose,
				null,
				this::getChassisSpeeds,
				this::drive,
				controller,
				SwerveConstants.robotConfig,
				() -> false,
				this);
		if (Utils.isSimulation()) startSimThread();

		System.out.println("Swerve Starting!");
	}

	private void drive(ChassisSpeeds robotSpeeds, DriveFeedforwards feedforward) {
		this.setControl(m_pathApplyRobotSpeeds.withSpeeds(robotSpeeds));
	}

	/**
	 * Returns a command that applies the specified control request to this swerve drivetrain.
	 *
	 * @param requestSupplier Function returning the request to apply
	 * @return Command to run
	 */
	public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
		return run(() -> this.setControl(requestSupplier.get())).withName("Drive Request");
	}

	/**
	 * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
	 * #m_sysIdRoutineToApply}.
	 *
	 * @param direction Direction of the SysId Quasistatic test
	 * @return Command to run
	 */
	public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
		return m_sysIdRoutineToApply.quasistatic(direction);
	}

	/**
	 * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
	 * #m_sysIdRoutineToApply}.
	 *
	 * @param direction Direction of the SysId Dynamic test
	 * @return Command to run
	 */
	public Command sysIdDynamic(SysIdRoutine.Direction direction) {
		return m_sysIdRoutineToApply.dynamic(direction);
	}

	@Override
	public void periodic() {
		/*
		 * Periodically try to apply the operator perspective.
		 * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
		 * This allows us to correct the perspective in case the robot code restarts mid-match.
		 * Otherwise, only check and apply the operator perspective if the DS is disabled.
		 * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
		 */
		if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
			DriverStation.getAlliance()
					.ifPresent(
							allianceColor -> {
								setOperatorPerspectiveForward(
										allianceColor == Alliance.Red
												? kRedAlliancePerspectiveRotation
												: kBlueAlliancePerspectiveRotation);
								m_hasAppliedOperatorPerspective = true;
							});
		}

		Pose2d pose = getPose();
		SmartDashboard.putNumber("Swerve/Pose x", pose.getX());
		SmartDashboard.putNumber("Swerve/Pose y", pose.getY());
		SmartDashboard.putNumber("Swerve/Rotation", pose.getRotation().getDegrees());
		//        System.out.println(this.getCurrentCommand().getName());
	}

	public Pose2d getPose() {
		return getState().Pose;
		return simDrivetrain == null
			? getState().Pose
			: simDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
	}

	// public void resetOdotoSim() {
	// 	if (simDrivetrain == null) return;
	// 	this.resetPose(simDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose());
	// }

	public ChassisSpeeds getChassisSpeeds() {
		return getState().Speeds;
	}

	/**
	 * Follows the given field-centric path sample with PID.
	 *
	 * @param sample Sample along the path to follow
	 */
	public void followPath(SwerveSample sample) {
		m_pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

		var pose = getPose();
		var targetSpeeds = sample.getChassisSpeeds();
		ChassisSpeeds speeds =
				new ChassisSpeeds(
						sample.vx + m_pathXController.calculate(pose.getX(), sample.x),
						sample.vy + m_pathYController.calculate(pose.getY(), sample.y),
						sample.omega
								+ m_pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading));

		setControl(m_pathApplyFieldSpeeds.withSpeeds(speeds));
		// setControl(
		//         m_pathApplyFieldSpeeds.withSpeeds(speeds)
		//                 .withWheelForceFeedforwardsX(sample.moduleForcesX())
		//                 .withWheelForceFeedforwardsY(sample.moduleForcesY())
		// );
	}

	public static MapleSimSwerveDrivetrain simDrivetrain = null;

	@Override
	public void resetPose(Pose2d pose) {
		if (simDrivetrain != null) {
		simDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
		Timer.delay(0.05); // Wait for simulation to update
		}
		super.resetPose(pose);
	}

	private void startSimThread() {
		simDrivetrain =
			new MapleSimSwerveDrivetrain(
				Units.Seconds.of(kSimLoopPeriod),
				Units.Pounds.of(110),
				Units.Inches.of(30),
				Units.Inches.of(30),
				DCMotor.getKrakenX60Foc(1),
				DCMotor.getKrakenX60Foc(1),
				1.2,
				getModuleLocations(),
				getPigeon2(),
				getModules(),
				TunerConstants.FrontLeft,
				TunerConstants.FrontRight,
				TunerConstants.BackLeft,
				TunerConstants.BackRight);

		m_simNotifier = new Notifier(simDrivetrain::update);
		m_simNotifier.startPeriodic(kSimLoopPeriod);
	}
}
