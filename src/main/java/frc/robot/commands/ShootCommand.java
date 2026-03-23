package frc.robot.commands;

import java.util.prefs.NodeChangeEvent;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferSubsystem;

public class ShootCommand extends Command {
  private final ShooterSubsystem shooter;
  private final TransferSubsystem transfer;
  private Boolean noDistance = false;

  public ShootCommand(ShooterSubsystem shooter, TransferSubsystem transfer) {
    this.shooter = shooter;
    this.transfer = transfer;
    addRequirements(shooter, transfer);
  }

  public ShootCommand(ShooterSubsystem shooter, TransferSubsystem transfer, Boolean noDistance) {
    this.shooter = shooter;
    this.transfer = transfer;
    this.noDistance = noDistance;
    addRequirements(shooter, transfer);
  }


  @Override
  public void initialize() {
    if (noDistance){
      shooter.requestAutoAimlessShoot();
    }else{
      shooter.requestShoot();
    }
    transfer.requestTransfer();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    shooter.requestIdle();
    transfer.requestIdle();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
