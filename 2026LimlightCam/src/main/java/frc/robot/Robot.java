package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.TunableParameter;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.util.ControlBoard;


public class Robot extends TimedRobot {
  // public static final CANBus riobus = new CANBus("rio");

   public static final CANBus drivebus = new CANBus("Drivebus");

  // private final ControlBoard controlBoard;
  // private final CommandScheduler scheduler;
  
 
  // public Robot() {
  //   scheduler = CommandScheduler.getInstance();

  //   ControlBoard tmpControlBoard = null;
  //   try {
  //     tmpControlBoard = ControlBoard.getInstance();
  //   } catch (Throwable t) {
  //     DriverStation.reportError("ControlBoard init failed: " + t.toString(), t.getStackTrace());
  //     t.printStackTrace();
  //   }
  //   this.controlBoard = tmpControlBoard;
  // }

  // @Override
  // public void robotInit() {
  //   System.out.println("Robot.robotInit() start");
  //   for (int port = 580; port <= 5809; port++) {
  //     PortForwarder.add(port, "limelight.local",port);
  //   }

  //   Pathfinding.setPathfinder(
  //     new LocalADStar()
  //   );

  //   scheduler.schedule(PathfindingCommand.warmupCommand());
  // }

  // @Override
  // public void robotPeriodic() {
  //   TunableParameter.updateAll();
  //   scheduler.run();
  //   SmartDashboard.putBoolean("Limelight has Target", LimelightSubsystem.getInstance().hasTarget());

  //   if (controlBoard != null) controlBoard.displayUI();
  // }

  // @Override
  // public void driverStationConnected() {
  //   ControlBoard.getInstance().tryInit();
  // }

  // @Override
  // public void disabledInit() {
  //   SignalLogger.stop();
  // }

  // @Override
  // public void disabledPeriodic() {}



  @Override
  public void teleopInit() {
    LimelightSubsystem.getInstance();
  }


  @Override
  public void teleopPeriodic() {
    edu.wpi.first.wpilibj2.command.CommandScheduler.getInstance().run();
  }




  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
