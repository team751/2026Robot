package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.transfer.TransferSubsystem;

/**
 * Briefly reverses the transfer to clear a jam, then restores whatever state the transfer was in
 * before — so if the transfer was mid-TRANSFER when the driver hit "unstuck" (D-pad left), it goes
 * right back to TRANSFER afterward instead of dropping to IDLE. This is what makes it "simple"
 * compared to a full jiggle sequence: it only touches the transfer, not intake/extender too.
 */
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
    // Keep re-capturing oldState as long as we're not the one who put it in REVERSE —
    // this only matters if something else changes the transfer's state while this command
    // is running, so end() restores the most recent non-reverse state, not a stale one.
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
