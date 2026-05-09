package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.transfer.TransferSubsystem;

public class SimpleUnstuck extends Command {
  TransferSubsystem transfer;
  TransferSubsystem.TransferState oldState;

  public SimpleUnstuck(TransferSubsystem transfer) {
    this.transfer = transfer;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    oldState = transfer.getState();
    transfer.requestReverse();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (transfer.getState() != TransferSubsystem.TransferState.REVERSE) {
      oldState = transfer.getState();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    transfer.requestState(oldState);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
