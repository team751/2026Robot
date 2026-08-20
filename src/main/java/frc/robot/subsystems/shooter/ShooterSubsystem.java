package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.util.FieldConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnFly;

/**
 * Runs the shooter: a main flywheel motor, a follower motor mounted opposed to it (so the two
 * wheels spin toward each other and pinch the game piece), and a small transfer motor that feeds
 * pieces into the flywheels once they're spinning. Unlike IntakeSubsystem's request-flag pattern,
 * {@code request*()} here sets {@link #state} directly — periodic() just reads it.
 *
 * <p>In simulation there's no real flywheel to launch a physical game piece, so {@link
 * #simulationPeriodic()} manually spawns a simulated projectile ({@link RebuiltFuelOnFly}) with a
 * fixed launch speed/angle whenever a shot is requested.
 */
public class ShooterSubsystem extends SubsystemBase {
  private static ShooterSubsystem instance;

  // Motors
  private final TalonFX flywheelMotor =
      ShooterConstants.flywheelMotorConfig.createDevice(TalonFX::new);
  private final VelocityVoltage flywheelControl = new VelocityVoltage(0);

  private final TalonFX followMotor = ShooterConstants.followMotorConfig.createDevice(TalonFX::new);
  private final Follower followControl =
      new Follower(ShooterConstants.flywheelMotorConfig.canID, MotorAlignmentValue.Opposed);

  private final TalonFX shooterTransferMotor =
      ShooterConstants.transferMotorConfig.createDevice(TalonFX::new);
  // private final Follower shooterTransferControl = new
  // Follower(ShooterConstants.flywheelMotorConfig.canID, MotorAlignmentValue.Aligned);
  private final VoltageOut shooterTransferControl = new VoltageOut(0);

  private double targetRPS;
  private double lastShotTime = 0.3;
  public Boolean isAuto = false;

  /* State Machine Logic */
  private enum ShooterState {
    IDLE,
    AASHOOT,
    REVERSE,
    SHOOT
  }

  private ShooterState state = ShooterState.IDLE;

  public static ShooterSubsystem getInstance() {
    if (instance == null) instance = new ShooterSubsystem();
    return instance;
  }

  private ShooterSubsystem() {
    flywheelMotor.setControl(flywheelControl);
    followMotor.setControl(followControl);
    shooterTransferMotor.setControl(shooterTransferControl);

    setShooterMotor(0);
  }

  @Override
  public void periodic() {
    switch (state) {
      case IDLE -> setShooterSpeed(0, 0);
      case AASHOOT -> setShooterSpeed(
          ShooterConstants.flywheelSpeed, ShooterConstants.transferVoltage);
      case REVERSE -> setTransferMotor(ShooterConstants.transferSpitVoltage);
      case SHOOT -> setShooterSpeed(calculateShooterSpeed(), ShooterConstants.transferVoltage);
    }
  }

  /** Runs just the main flywheel motor */
  private void setShooterMotor(double flywheelVelocity) {
    flywheelMotor.setControl(flywheelControl.withVelocity(flywheelVelocity));
    targetRPS = flywheelVelocity;
  }

  /** Runs just the transfer motor on shooter */
  // TODO: change to private, made public for testing stuff
  public void setTransferMotor(double transferVoltage) {
    shooterTransferMotor.setControl(shooterTransferControl.withOutput(transferVoltage));
  }

  /**
   * Runs both the main shooter motor and transfer motor. The transfer only actually feeds a piece
   * in once the flywheel is within 0.1 RPS of its target speed — feeding too early would launch the
   * piece at the wrong speed. {@code isAuto} bypasses that wait entirely during autonomous, trading
   * shot consistency for not stalling out a scripted auto routine.
   */
  private void setShooterSpeed(double flywheelVelocity, double transferVoltage) {
    flywheelMotor.setControl(flywheelControl.withVelocity(flywheelVelocity));
    if (Math.abs(flywheelVelocity - flywheelMotor.getVelocity().getValueAsDouble()) < 0.1
        || isAuto) {
      shooterTransferMotor.setControl(shooterTransferControl.withOutput(transferVoltage));
    }
  }

  /** Straight-line distance from the robot to the current alliance's hub, in centimeters. */
  private double getRobotDistanceFromHub() {
    SwerveSubsystem swerve = SwerveSubsystem.getInstance();
    Pose2d hubPose = FieldConstants.getAllianceHub();
    Pose2d robotPose = swerve.getPose();
    return 100 * Math.hypot(hubPose.getX() - robotPose.getX(), hubPose.getY() - robotPose.getY());
  }

  /** Whether the robot is within the distance range the speed curve was tuned for. */
  private boolean canShoot() {
    double distanceCM = getRobotDistanceFromHub();
    return distanceCM >= ShooterConstants.minShootingDistance
        && distanceCM <= ShooterConstants.maxShootingDistance;
  }

  /**
   * Picks a flywheel RPS from distance to the hub. Outside the tuned distance range ({@link
   * #canShoot} is false), falls back to the fixed {@link ShooterConstants#flywheelSpeed} rather
   * than extrapolating the curve, since the linear fit isn't trustworthy far outside the range it
   * was measured over.
   */
  private double calculateShooterSpeed() {
    if (!canShoot()) {
      return ShooterConstants.flywheelSpeed;
    }

    double distanceCM = getRobotDistanceFromHub();
    return ((distanceCM - ShooterConstants.shooterDistanceCurveYIntercept)
            / ShooterConstants.shooterDistanceCurveSlope)
        - 1.7;
  }

  public void requestIdle() {
    state = ShooterState.IDLE;
  }

  public void requestAutoAimlessShoot() {
    state = ShooterState.AASHOOT;
  }

  public void requestShoot() {
    state = ShooterState.SHOOT;
  }

  public void requestSpit() {
    state = ShooterState.REVERSE;
  }

  public void simulationPeriodic() {
    if (state == ShooterState.SHOOT || state == ShooterState.AASHOOT) {
      // Check cooldown - must wait 0.25 seconds between shots
      // (comment says 0.25s but the actual check below uses 0.3s — trust the code, not the
      // comment, if you're trying to figure out the real cooldown)
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
      RebuiltFuelOnFly fuelOnFly =
          new RebuiltFuelOnFly(
              // Specify the position of the chassis when the fuel is launched
              SwerveSubsystem.simDrivetrain
                  .mapleSimDrive
                  .getSimulatedDriveTrainPose()
                  .getTranslation(),
              // Specify the translation of the shooter from the robot center (in the
              // shooter's reference frame)
              new Translation2d(0, 0),
              // Specify the field-relative speed of the chassis, adding it to the initial
              // velocity of the projectile
              SwerveSubsystem.simDrivetrain.mapleSimDrive
                  .getDriveTrainSimulatedChassisSpeedsRobotRelative(),
              // The shooter facing direction is the same as the robot's facing direction
              SwerveSubsystem.simDrivetrain
                  .mapleSimDrive
                  .getSimulatedDriveTrainPose()
                  .getRotation(),
              // Initial height of the flying fuel
              Meters.of(1),
              // The launch speed is proportional to the RPM; assumed to be 16 meters/second
              // at 6000 RPM
              MetersPerSecond.of(8.35),
              // The angle at which the fuel is launched
              Radians.of(1.4));

      SimulatedArena.getInstance().addGamePieceProjectile(fuelOnFly);

      // Update last shot time for cooldown tracking
      lastShotTime = currentTime;
    }
    ;
  }
}
