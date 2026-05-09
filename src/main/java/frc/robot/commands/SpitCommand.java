package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.ExtenderSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.transfer.TransferSubsystem;

public class SpitCommand extends Command {
  private final TransferSubsystem transfer;
  private final IntakeSubsystem intake;
  private final ExtenderSubsystem extender;

  public SpitCommand(
      TransferSubsystem transfer, IntakeSubsystem intake, ExtenderSubsystem extender) {
    this.transfer = transfer;
    this.intake = intake;
    this.extender = extender;
    addRequirements(transfer, intake, extender);
  }

  @Override
  public void initialize() {
    transfer.requestReverse();
    intake.requestSpit();
    extender.requestExtend();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {
    transfer.requestIdle();
    intake.requestIdle();
    extender.requestRetract();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
