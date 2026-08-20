package frc.robot.subsystems.drive;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.util.DriveFeedforwards;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.generated.TunerConstants;
import frc.robot.subsystems.drive.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.simulation.MapleSimSwerveDrivetrain;
import java.util.function.Supplier;

/**
 * The actual swerve drivetrain. Extends CTRE's generated {@code TunerSwerveDrivetrain} (built from
 * Tuner X's config — see {@link TunerConstants}) and layers on: PathPlanner's AutoBuilder setup (so
 * autonomous paths can drive this drivetrain), alliance-aware "operator perspective" handling (so
 * joystick-forward always means away from your own wall), tilt/speed stability checks, and — only
 * in simulation — a MapleSim physics thread that makes {@link #getPose()} return a physically
 * simulated pose instead of pure wheel-odometry math.
 */
public class SwerveSubsystem extends TunerSwerveDrivetrain implements Subsystem {
  // Rotation2d.kZero;
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  private boolean m_hasAppliedOperatorPerspective = false;
  // Tracks the currently-applied operator perspective rotation so we can adjust odometry
  // when the perspective changes (keeps "forward" consistent when switching alliances).
  private Rotation2d m_operatorPerspectiveRotation = Rotation2d.kZero;

  private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();

  private final SysIdRoutine m_sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null,
              Units.Volts.of(7),
              null,
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(m_steerCharacterization.withVolts(volts)), null, this));

  private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds()
          .withSteerRequestType(SwerveModule.SteerRequestType.Position)
          .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

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

  public SwerveSubsystem(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {

    super(
        drivetrainConstants,
        MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(modules));

    CommandScheduler.getInstance().registerSubsystem(this);

    // Configure AutoBuilder HERE (remove from Robot.java)
    // These PID gains are what PathPlanner uses to correct the robot back onto its planned
    // path during autonomous — separate from (and not related to) the driver's manual
    // auto-aim/axis-align PID controllers in ControlBoard.
    PathFollowingController controller =
        new PPHolonomicDriveController(
            new PIDConstants(0.5, 0.0, 0.04), // Translation PID
            new PIDConstants(4.75, 0.0, 0.0) // Rotation PID
            );

    // Wires this drivetrain up to PathPlanner: how to read our pose, reset it, read our
    // current speeds, and command new speeds, plus which alliance to mirror paths for.
    AutoBuilder.configure(
        this::getPose, // Robot pose supplier
        this::resetPose, // Method to reset odometry
        this::getRobotRelativeSpeeds, // ChassisSpeeds supplier
        this::drive, // Method to drive the robot (now accepts DriveFeedforwards)
        controller, // Path following controller
        SwerveConstants.robotConfig, // RobotConfig
        () -> {
          // Boolean supplier for alliance color
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this);

    // System.out.println("Swerve Starting!");

    // Try to apply operator perspective as early as possible so a deploy (process restart)
    // gets the same perspective as a full power-cycle. We also publish telemetry so we can
    // see what perspective was applied on deploy vs power-cycle.
    DriverStation.getAlliance()
        .ifPresent(
            allianceColor -> {
              var rot =
                  allianceColor == Alliance.Red
                      ? kRedAlliancePerspectiveRotation
                      : kBlueAlliancePerspectiveRotation;
              // Use helper that adjusts odometry pose when perspective changes so the
              // operator's forward stays consistent.
              setOperatorPerspectiveAndAdjustPose(rot);
            });

    // Start simulation thread if running in simulation
    if (Utils.isSimulation()) startSimThread();
  }

  /** Current robot-relative chassis speeds, computed from each module's actual state. */
  public ChassisSpeeds getRobotRelativeSpeeds() {
    return getKinematics().toChassisSpeeds(getState().ModuleStates);
  }

  // This method now accepts DriveFeedforwards as PathPlanner expects
  // Called by PathPlanner every loop during autonomous to drive the robot along the path;
  // feedForward is unused here since ApplyRobotSpeeds handles velocity control internally.
  private void drive(ChassisSpeeds robotSpeeds, DriveFeedforwards feedForward) {
    this.setControl(m_pathApplyRobotSpeeds.withSpeeds(robotSpeeds));
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get())).withName("Drive Request");
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    // if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
    //   if (DriverStation.getAlliance().isPresent()) {
    //     var rot = kBlueAlliancePerspectiveRotation;
    //     if (DriverStation.getAlliance().get() == Alliance.Red) {
    //       rot = kRedAlliancePerspectiveRotation;
    //     }

    //     setOperatorPerspectiveAndAdjustPose(rot);
    //   }

    //   // DriverStation.getAlliance()
    //   //     .ifPresent(
    //   //         allianceColor -> {
    //   //             var rot =
    //   //                 allianceColor == Alliance.Red
    //   //                 ? kRedAlliancePerspectiveRotation
    //   //                 : kBlueAlliancePerspectiveRotation;
    //   //             setOperatorPerspectiveForward(rot);
    //   //             m_hasAppliedOperatorPerspective = true;
    //   //             SmartDashboard.putNumber("OperatorPerspectiveDeg", rot.getDegrees());
    //   //             SmartDashboard.putString("OperatorPerspectiveAlliance",
    // allianceColor.name());
    //   //         }
    //   //     );
    // }

    // Shows where SwerveDrive thinks the robot is positioned on the field.
    Pose2d pose = getPose();
    SmartDashboard.putNumber("Swerve/Pose x", pose.getX());
    SmartDashboard.putNumber("Swerve/Pose y", pose.getY());
    SmartDashboard.putNumber("Swerve/Rotation", pose.getRotation().getDegrees());
  }

  /**
   * Current robot pose. On real hardware this comes from CTRE's own wheel+vision odometry fusion;
   * in simulation it comes from MapleSim's physics engine instead, since the real odometry math has
   * nothing physical to track against in sim.
   */
  public Pose2d getPose() {
    return simDrivetrain == null
        ? getState().Pose
        : simDrivetrain.mapleSimDrive.getSimulatedDriveTrainPose();
  }

  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  public ChassisSpeeds getChassisSpeeds() {
    return getState().Speeds;
  }

  // IMU stability thresholds — used to tell whether the robot is sitting flat and still
  // (e.g. not tipping, not currently driving hard) vs. actively moving/tilting. Not
  // currently consumed anywhere in this codebase, but available for anything that needs
  // to know "is the robot safe to trust right now" (e.g. before a climb sequence).
  private static final double PITCH_STABLE_DEG = 5.0;
  private static final double ROLL_STABLE_DEG = 5.0;
  private static final double TILT_RATE_STABLE_DPS = 10.0;
  private static final double SPEED_STABLE_MPS = 0.5;

  public boolean getPitchStable() {
    double pitch = Math.abs(getPigeon2().getPitch().getValueAsDouble());
    double pitchRate = Math.abs(getPigeon2().getAngularVelocityYWorld().getValueAsDouble());
    return pitch < PITCH_STABLE_DEG && pitchRate < TILT_RATE_STABLE_DPS;
  }

  public boolean getRollStable() {
    double roll = Math.abs(getPigeon2().getRoll().getValueAsDouble());
    double rollRate = Math.abs(getPigeon2().getAngularVelocityXWorld().getValueAsDouble());
    return roll < ROLL_STABLE_DEG && rollRate < TILT_RATE_STABLE_DPS;
  }

  public boolean getStable() {
    ChassisSpeeds speeds = getChassisSpeeds();
    double linearSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    return getPitchStable() && getRollStable() && linearSpeed < SPEED_STABLE_MPS;
  }

  @Override
  public void resetPose(Pose2d pose) {
    // In sim, the physics engine also needs to be told the new pose, not just CTRE's
    // internal odometry — otherwise getPose() (which reads from MapleSim, see above)
    // would disagree with what we just reset to.
    if (simDrivetrain != null) {
      simDrivetrain.mapleSimDrive.setSimulationWorldPose(pose);
      Timer.delay(0.05); // Wait for simulation to update
    }
    super.resetPose(pose);
  }

  public void setRobotRotationByAlliance() {
    if (DriverStation.getAlliance().isPresent()) {
      var rot = kBlueAlliancePerspectiveRotation;
      if (DriverStation.getAlliance().get() == Alliance.Red) {
        rot = kRedAlliancePerspectiveRotation;
      }
      resetPose(new Pose2d(0, 0, rot));
      setOperatorPerspectiveAndAdjustPose(rot);
    }
  }

  /**
   * Set the operator perspective and rotate the odometry pose by the same delta so that "forward"
   * from the operator's perspective stays consistent when the perspective (alliance) changes.
   */
  public void setOperatorPerspectiveAndAdjustPose(Rotation2d newRot) {
    if (newRot == null) return;

    // If no change, still apply underlying method but do nothing to pose
    double eps = 1e-6;
    double oldRad = m_operatorPerspectiveRotation.getRadians();
    double newRad = newRot.getRadians();
    if (Math.abs(newRad - oldRad) < eps) {
      setOperatorPerspectiveForward(newRot);
      m_hasAppliedOperatorPerspective = true;
      return;
    }

    // Compute the delta rotation to apply to the existing pose
    double delta = newRad - oldRad;
    Rotation2d deltaRot = new Rotation2d(delta);

    Pose2d currentPose = getPose();
    Pose2d adjusted =
        new Pose2d(currentPose.getTranslation(), currentPose.getRotation().rotateBy(deltaRot));

    // Apply the operator perspective and reset odometry to the adjusted pose
    setOperatorPerspectiveForward(newRot);
    resetPose(adjusted);

    m_operatorPerspectiveRotation = newRot;
    m_hasAppliedOperatorPerspective = true;

    SmartDashboard.putNumber("OperatorPerspectiveDeg", newRot.getDegrees());
    // SmartDashboard.putString(
    //     "OperatorPerspectiveAlliance",
    //     DriverStation.getAlliance().isPresent()
    //         ? DriverStation.getAlliance().get().name()
    //         : "Unknown");
  }

  // Simulation support
  public static MapleSimSwerveDrivetrain simDrivetrain = null;
  private Notifier m_simNotifier = null;
  private static final double kSimLoopPeriod = 0.005;

  /**
   * Only called in simulation (see the constructor). Spins up MapleSim's physics engine on a
   * separate {@link Notifier} timer running at 200Hz ({@link #kSimLoopPeriod} = 5ms) — much faster
   * than the normal 50Hz robot loop — since physics simulation (collisions, wheel slip, etc) needs
   * a tighter timestep to stay numerically stable than the main control loop does.
   */
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
