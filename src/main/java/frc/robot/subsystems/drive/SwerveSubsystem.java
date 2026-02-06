package frc.robot.subsystems.drive;

import java.util.function.Supplier;

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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.drive.generated.TunerConstants;
import frc.robot.subsystems.drive.generated.TunerConstants.TunerSwerveDrivetrain;

/* TODO: SwerveDrive Subsystem Overview/Explanation
 * SwerveDrive is the main file for controlling how the robot drives.
 * Our robot is field centric, not robot centric. Field centric means
 * that whenever you want to go forward (using the joystick) the robot will
 * go forward in the direction YOU are facing, regardless of the robot's rotation.
 * 
 * Robot centric is much worse, since if you imagine your robot is facing to the left
 * and you push forward on the joystick, the robot would actually go left, since 
 * thats the front for the robot. 
 * 
 * Hopefully that was a good explanation .-.
 * 
 * 
 * Anyways, this file sets that up and also sets up stuff for auton (talk to alexander for an explanation)
 * I suggest you also take a look at generated/TunerConstants.java as well.
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


    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                Units.Volts.of(7),
                null,
                state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
                ),
                new SysIdRoutine.Mechanism(
                    volts -> setControl(
                        m_steerCharacterization.withVolts(volts)
                    ),
                    null,
                    this));
    
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
                    TunerConstants.BackRight
                );
        }
        return instance;
    }

    public SwerveSubsystem(
        SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules
    ) {

        super(drivetrainConstants, modules);
        
        CommandScheduler.getInstance().registerSubsystem(this);
        
        // Configure AutoBuilder HERE (remove from Robot.java)
        PathFollowingController controller = 
            new PPHolonomicDriveController(
                new PIDConstants(0.5, 0.0, 0.0),  // Translation PID
                new PIDConstants(0.3, 0.0, 0.0)       // Rotation PID
            );
        
        AutoBuilder.configure(
            this::getPose,                  // Robot pose supplier
            this::resetPose,                // Method to reset odometry
            this::getRobotRelativeSpeeds,         // ChassisSpeeds supplier
            this::drive,                    // Method to drive the robot (now accepts DriveFeedforwards)
            controller,                     // Path following controller
            SwerveConstants.robotConfig,    // RobotConfig
            () -> {
                // Boolean supplier for alliance color
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this                           
        );
        
    System.out.println("Swerve Starting!");

    // Try to apply operator perspective as early as possible so a deploy (process restart)
    // gets the same perspective as a full power-cycle. We also publish telemetry so we can
    // see what perspective was applied on deploy vs power-cycle.
    DriverStation.getAlliance()
        .ifPresent(
            allianceColor -> {
                var rot = allianceColor == Alliance.Red
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation;
                // Use helper that adjusts odometry pose when perspective changes so the
                // operator's forward stays consistent.
                setOperatorPerspectiveAndAdjustPose(rot);
            });
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
                return getKinematics().toChassisSpeeds(getState().ModuleStates);
        }

    // This method now accepts DriveFeedforwards as PathPlanner expects
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
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            if (DriverStation.getAlliance().isPresent()) {
                var rot = kBlueAlliancePerspectiveRotation;
                if (DriverStation.getAlliance().get() == Alliance.Red) {
                    rot = kRedAlliancePerspectiveRotation;
                }

                setOperatorPerspectiveAndAdjustPose(rot);
            }

            // DriverStation.getAlliance()
            //     .ifPresent(
            //         allianceColor -> {
            //             var rot =
            //                 allianceColor == Alliance.Red
            //                 ? kRedAlliancePerspectiveRotation
            //                 : kBlueAlliancePerspectiveRotation;
            //             setOperatorPerspectiveForward(rot);
            //             m_hasAppliedOperatorPerspective = true;
            //             SmartDashboard.putNumber("OperatorPerspectiveDeg", rot.getDegrees());
            //             SmartDashboard.putString("OperatorPerspectiveAlliance", allianceColor.name());
            //         }
            //     );
        }

        // Shows where SwerveDrive thinks the robot is positioned on the field.
        Pose2d pose = getPose();
        SmartDashboard.putNumber("Swerve/Pose x", pose.getX());
        SmartDashboard.putNumber("Swerve/Pose y", pose.getY());
        SmartDashboard.putNumber("Swerve/Rotation", pose.getRotation().getDegrees());
    }

    public Pose2d getPose() {
        return getState().Pose;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return getState().Speeds;
    }

    @Override
    public void resetPose(Pose2d pose) {
        super.resetPose(pose);
    }

    public void bigResetPose() {
        this.resetPose(new Pose2d(0.0, 0.0, this.getPigeon2().getRotation2d()));
    }

    /**
     * Set the operator perspective and rotate the odometry pose by the same delta so that
     * "forward" from the operator's perspective stays consistent when the perspective
     * (alliance) changes.
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
        Pose2d adjusted = new Pose2d(currentPose.getTranslation(), currentPose.getRotation().rotateBy(deltaRot));

        // Apply the operator perspective and reset odometry to the adjusted pose
        setOperatorPerspectiveForward(newRot);
        resetPose(adjusted);

        m_operatorPerspectiveRotation = newRot;
        m_hasAppliedOperatorPerspective = true;

        SmartDashboard.putNumber("OperatorPerspectiveDeg", newRot.getDegrees());
        SmartDashboard.putString("OperatorPerspectiveAlliance",
                DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get().name() : "Unknown");
    }
}
