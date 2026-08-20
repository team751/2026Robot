package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Odometry;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
// import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.util.ControlBoard;

/**
 * The top-level robot class. WPILib calls the lifecycle methods below (robotInit, teleopInit, etc)
 * automatically as the match progresses — see docs/architecture.md for the full initialization
 * order and why it matters (e.g. Odometry/LimelightSubsystem aren't created until {@link
 * #driverStationConnected()}, once we actually know our alliance color).
 *
 * <p>{@code robotPeriodic()} runs every 20ms (50Hz) regardless of mode, and is where the {@link
 * CommandScheduler} actually runs all subsystem periodic() methods and active commands.
 */
public class Robot extends TimedRobot {
  /** CANBus only used for climber */
  public static final CANBus riobus = new CANBus("rio");

  /** CANBus only used for swerve */
  public static final CANBus drivebus = new CANBus("drivebus");

  /** CANBus used for everything but climber and swerve */
  public static final CANBus gamepiecebus = new CANBus("gamepiecebus");

  private final ControlBoard controlBoard;
  private final CommandScheduler scheduler;
  private SwerveSubsystem swerve;
  private RobotContainer robotContainer;

  private Command autonomousCommand;

  public Robot() {
    // Odometry.getInstance();
    scheduler = CommandScheduler.getInstance();
    swerve = SwerveSubsystem.getInstance();

    // ControlBoard's constructor builds PS5Controller objects, which can throw if a
    // controller isn't plugged in / recognized. Wrapped in try/catch so a missing
    // controller doesn't crash the whole robot program before code even starts running.
    ControlBoard tmpControlBoard = null;
    try {
      tmpControlBoard = ControlBoard.getInstance();
    } catch (Throwable t) {
      DriverStation.reportError("ControlBoard init failed: " + t.toString(), t.getStackTrace());
      // t.printStackTrace();
    }
    this.controlBoard = tmpControlBoard;
  }

  @Override
  public void robotInit() {
    // System.out.println("Robot.robotInit() start");
    // for (int port = 5800; port <= 5809; port++) {
    //   PortForwarder.add(port, "limelight.local", port);
    // }
    // Stream
    // PortForwarder.add(5800, "10.7.51.71",5800);
    // PortForwarder.add(5800, "10.7.51.75",5800);

    // Dashboard
    // PortForwarder.add(5801, "10.7.51.71",5801);
    // PortForwarder.add(5801, "10.7.51.75",5801);
    robotContainer = new RobotContainer();
    // Sets up the two Limelight cameras (offsets, names) — see LimelightSubsystem.initLimsplz().
    LimelightSubsystem.getInstance().initLimsplz();
    // Default to blue alliance until the FMS/Driver Station tells us otherwise.
    controlBoard.isBlue =
        !DriverStation.getAlliance().isPresent()
            || DriverStation.getAlliance().get() != Alliance.Red;
  }

  @Override
  public void robotPeriodic() {
    // TunableParameter.updateAll();
    try {
      // scheduler.run() is the heart of the robot: it runs every subsystem's periodic()
      // and steps every currently-scheduled Command once. Nothing subsystem-related
      // happens unless it's called from here.
      scheduler.run();
    } catch (Throwable t) {
      // A crash inside any one command/subsystem would normally kill the whole robot
      // program (and every other subsystem with it). Catching here means one bug in,
      // say, the shooter, doesn't also take down driving.
      DriverStation.reportError(
          "Unhandled exception in CommandScheduler: " + t.toString(), t.getStackTrace());
      // t.printStackTrace();
    }
    if (controlBoard != null) controlBoard.displayUI();
  }

  @Override
  public void driverStationConnected() {
    // Runs once the Driver Station is actually connected, which is the first point
    // we can reliably know our alliance color — Odometry/vision setup depends on this,
    // so it's created here (see class-level doc comment) rather than in robotInit().
    ControlBoard.getInstance().tryInit();
  }

  @Override
  public void disabledInit() {
    SignalLogger.stop();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    // isAuto relaxes ShooterSubsystem's "only feed the transfer once flywheel is up to
    // speed" check — see ShooterSubsystem.setShooterSpeed().
    ShooterSubsystem.getInstance().isAuto = true;
    // Grabs whichever auto routine is selected in the Shuffleboard/Elastic dropdown
    // (see RobotContainer's autoChooser) and schedules it to actually run.
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopInit() {
    Odometry.getInstance();
    ShooterSubsystem.getInstance().isAuto = false;
    controlBoard.isBlue =
        !DriverStation.getAlliance().isPresent()
            || DriverStation.getAlliance().get() != Alliance.Red;
    // LimelightSubsystem.getInstance();
    // ClimberSubsystem.getInstance().zeroClimber();

    // "Forward" on the field is always +X toward the red wall in WPILib's coordinate
    // system, but we want the driver's stick-forward to always push the robot away from
    // their own alliance wall. Flipping the operator perspective 180° on red alliance
    // makes that true without the driver having to think about which wall they're on.
    var rot = Rotation2d.kZero;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
      rot = Rotation2d.k180deg;
    }
    swerve.setOperatorPerspectiveAndAdjustPose(rot);
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
