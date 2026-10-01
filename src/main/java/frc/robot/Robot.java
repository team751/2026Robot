package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.drive.Odometry;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.vision.PhotonVisionConstants;
import frc.robot.subsystems.vision.PhotonVisionSim;
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
    // Record all NetworkTables data (+ DS/joystick data) to a .wpilog on the RIO
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());

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
    // PhotonVision web UI at http://roborio-751-frc.local:5800 when tethered over USB.
    // (Only the UI: camera preview streams use other ports, so calibrate over the radio/Ethernet.)
    if (isReal()) {
      PortForwarder.add(5800, PhotonVisionConstants.COPROCESSOR_IP, 5800);
    }

    robotContainer = new RobotContainer();

    // Vision has to exist BEFORE the first scheduler loop so it fuses while disabled and in auto
    // (resetOdom:false autos depend on vision seeding the pose pre-match). Odometry.getInstance()
    // also creates PhotonVisionSubsystem, first, so cameras are read before Odometry fuses them.
    Odometry.getInstance();
    if (Utils.isSimulation()) {
      PhotonVisionSim.getInstance();
    }
    controlBoard.isBlue =
        !DriverStation.getAlliance().isPresent()
            || DriverStation.getAlliance().get() != Alliance.Red;
  }

  @Override
  public void robotPeriodic() {
    // TunableParameter.updateAll();
    try {
      // Threads.setCurrentThreadPriority(true, 6);
      scheduler.run();
      // Threads.setCurrentThreadPriority(false, 0);
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
    ShooterSubsystem.getInstance().isAuto = true;
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
    ShooterSubsystem.getInstance().isAuto = false;
    controlBoard.isBlue =
        !DriverStation.getAlliance().isPresent()
            || DriverStation.getAlliance().get() != Alliance.Red;
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
