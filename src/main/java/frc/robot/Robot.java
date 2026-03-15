package frc.robot;

// TODO: stuff to test on the robot
// Transfer first:
// Nothing jams, etc
//
// Shooter:
// more calibration?
//
// Drive:
// moving w transfer and see if it still transfers
// intaking many balls at once
// shooting while moving(?)
// maybe possibly ram intake into something while extended??
//
// Limelight:
// calibrate and get it ready
// orientation and offset - side camera done

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
// import frc.robot.subsystems.climber.ClimberSubsystem;
// import frc.robot.subsystems.drive.Odometry;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.TunableParameter;
import frc.robot.subsystems.drive.Odometry;
import frc.robot.subsystems.drive.SwerveSubsystem;
// import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.util.ControlBoard;

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
    Odometry.getInstance();
    scheduler = CommandScheduler.getInstance();
    swerve = SwerveSubsystem.getInstance();

    ControlBoard tmpControlBoard = null;
    try {
      tmpControlBoard = ControlBoard.getInstance();
    } catch (Throwable t) {
      DriverStation.reportError("ControlBoard init failed: " + t.toString(), t.getStackTrace());
      t.printStackTrace();
    }
    this.controlBoard = tmpControlBoard;
  }

  @Override
  public void robotInit() {
    System.out.println("Robot.robotInit() start");
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local", port);
    }
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    TunableParameter.updateAll();
    try {
      scheduler.run();
    } catch (Throwable t) {
      DriverStation.reportError(
          "Unhandled exception in CommandScheduler: " + t.toString(), t.getStackTrace());
      t.printStackTrace();
    }
    if (controlBoard != null) controlBoard.displayUI();
  }

  @Override
  public void driverStationConnected() {
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
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
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
    // LimelightSubsystem.getInstance();
    // ClimberSubsystem.getInstance().zeroClimber();

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
