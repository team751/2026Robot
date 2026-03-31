package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Odometry;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
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
    //Odometry.getInstance();
    scheduler = CommandScheduler.getInstance();
    swerve = SwerveSubsystem.getInstance();

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
    PortForwarder.add(5800, "10.7.51.71",5800);
    PortForwarder.add(5800, "10.7.51.75",5800);

    // Dashboard
    PortForwarder.add(5801, "10.7.51.71",5801);
    PortForwarder.add(5801, "10.7.51.75",5801);
    robotContainer = new RobotContainer();
    LimelightSubsystem.getInstance().initLimsplz();
  }

  @Override
  public void robotPeriodic() {
    // TunableParameter.updateAll();
    try {
      //Threads.setCurrentThreadPriority(true, 6);
      scheduler.run();
      //Threads.setCurrentThreadPriority(false, 0);
    } catch (Throwable t) {
      DriverStation.reportError(
          "Unhandled exception in CommandScheduler: " + t.toString(), t.getStackTrace());
      // t.printStackTrace();
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
