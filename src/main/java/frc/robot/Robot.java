package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.TunableParameter;
import frc.robot.subsystems.drive.Odometry;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.vision.LimelightSubsystem;
import frc.robot.util.ControlBoard;

/* TODO: Robot.java """Explanation"""
 * Robot.java sets stuff up. It initializes all the subsystems and 
 * everything that you need for the robot to function as intended
 * 
 * If you're new to FRC code or the team: Hi! Welcome to the software team
 * its... fun... here... :D
 * 
 * Hopefully you know Java already (you probably won't understand most of the comments I've made if not),
 * since I added a """ton""" of comments """explaining""" what everything does!
 * 
 * Before you go, I'll TRY to explain how everything works here!
 * 
 * So the way that most, if not ALL, robots are structured in FRC is with Subsystems!
 * Subsystems are. well. What they say they are! They smaller systems that apart of
 * the larger robot (system). For example, Intake or Climber are subsystems. 
 * 
 * This makes it wayyy easier to program and keeps the code organized (one(I) might say even TOO organized).
 * These subsystems have their own code for setting up their motors and any electronic pieces that they
 * might need to function properly.
 * 
 * This is still a WIP and will be continued later :)
 * 
 */


public class Robot extends TimedRobot {
  public static final CANBus riobus = new CANBus("rio");

   public static final CANBus drivebus = new CANBus("Drivebus");

  private final ControlBoard controlBoard;
  private final CommandScheduler scheduler;
  private Odometry odometry;
  private SwerveSubsystem swerve;
  private RobotConfig config;
  
  private Command autonomousCommand;
  private SendableChooser<Command> autoChooser;
 
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
    this.controlBoard = tmpControlBoard; }




  @Override
  public void robotInit() {
    System.out.println("Robot.robotInit() start");
    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port, "limelight.local",port);
    }
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  @Override
  public void robotPeriodic() {
    TunableParameter.updateAll();
    try {
      scheduler.run();
    } catch (Throwable t) {
      DriverStation.reportError("Unhandled exception in CommandScheduler: " + t.toString(), t.getStackTrace());
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
  public void autonomousInit() {
    // Get the selected auto command from the chooser
    autonomousCommand = autoChooser.getSelected();
    
    // Schedule the autonomous command
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }




  @Override
  public void teleopInit() {
    LimelightSubsystem.getInstance();

    var rot = Rotation2d.kZero;
    if (DriverStation.getAlliance().get() == Alliance.Red) {
        rot = Rotation2d.k180deg;
    }

  // Apply operator perspective and adjust odometry so "forward" remains consistent
  // when switching alliances.
  swerve.setOperatorPerspectiveAndAdjustPose(rot);
  }

  @Override
  public void teleopPeriodic() {}




  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
