package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.TunableParameter;
import frc.robot.subsystems.motor.Motor;
import frc.robot.util.ControlBoard;

public class Robot extends TimedRobot {
  public static final CANBus riobus = new CANBus("rio");
  private final CommandScheduler scheduler;
  private final ControlBoard controlBoard;

  private final Motor motor;

  public Robot() {
    scheduler = CommandScheduler.getInstance();
    motor = Motor.getInstance();

    ControlBoard tmpControlBoard = null;
    try {
      tmpControlBoard = ControlBoard.getInstance();
    } catch (Throwable t) {
      DriverStation.reportError("it breaked: " + t.toString(), t.getStackTrace());
    }
    this.controlBoard = tmpControlBoard;
  }

  @Override
  public void robotPeriodic() {
    TunableParameter.updateAll();
    try {
      scheduler.run();
    } catch (Throwable t) {
      DriverStation.reportError("something broke in CommandScheduler or smth idk: " + t.toString(), t.getStackTrace());
      t.printStackTrace();
    }
    CommandScheduler.getInstance().run();
    if (controlBoard != null) controlBoard.displayUI();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {

   }

  @Override
  public void robotInit(){

  }

  @Override
  public void teleopPeriodic() {}


}
