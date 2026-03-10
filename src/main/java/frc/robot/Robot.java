package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.lib.TunableParameter;
import frc.robot.subsystems.climber.ClimberSubsystem;
// import frc.robot.subsystems.drive.Odometry;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
//import frc.robot.subsystems.vision.LimelightSubsystem;
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

  private Command autonomousCommand;
  private SendableChooser<Command> autoChooser;


  // COMMANDS FOR NAMED COMMANDS
  StartEndCommand shoot = new StartEndCommand(() -> ShooterSubsystem.getInstance().requestShoot(), () -> ShooterSubsystem.getInstance().requestIdle());
  InstantCommand stopShoot = new InstantCommand(() -> shoot.cancel());

  // InstantCommand [name] = new InstantCommand(() -> [method]);
  // StartEndCommand [name] = new StartEndCommand(() -> [method to run on start], () -> [method to run on end]);



  public Robot() {
    // Odometry.getInstance();
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

    NamedCommands.registerCommand("shoot", shoot);
    NamedCommands.registerCommand("stopShoot", stopShoot);



    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    ClimberSubsystem.getInstance().zeroClimber();

    
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
    autonomousCommand = autoChooser.getSelected();
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
    ClimberSubsystem.getInstance().zeroClimber();

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
