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

public class SwerveSubsystem extends TunerSwerveDrivetrain implements Subsystem {
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    private boolean m_hasAppliedOperatorPerspective = false;

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

        super(
            drivetrainConstants, modules);
        
        PathFollowingController controller = 
            new PPHolonomicDriveController(
                new PIDConstants(7.51, 0.0, 0.0), new PIDConstants(1,0.0,0.0));
        
        CommandScheduler.getInstance().registerSubsystem(this);;
        AutoBuilder.configure(
            this::getPose,
            null,
            this::getChassisSpeeds,
            this::drive,
            controller,
            SwerveConstants.robotConfig,
            () -> false,
            this);
        
        System.out.println("Swerve Starting!");
    }

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
            DriverStation.getAlliance()
                .ifPresent(
                    allianceColor -> {
                        setOperatorPerspectiveForward(
                            allianceColor == Alliance.Red
                            ? kRedAlliancePerspectiveRotation
                            : kBlueAlliancePerspectiveRotation);
                        m_hasAppliedOperatorPerspective = true;
                    }
                );
        }

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


}
