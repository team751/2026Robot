package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.ExtenderSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends Command {
  private final IntakeSubsystem intake;
  private final ExtenderSubsystem extender;

  public IntakeCommand(IntakeSubsystem intake, ExtenderSubsystem extender) {
    this.intake = intake;
    this.extender = extender;
    addRequirements(intake, extender);
  }

  @Override
  public void initialize() {
    extender.requestExtend();
  }

  @Override
  public void execute() {
    if (extender.isExtended()) {
      intake.requestIntake();
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.requestIdle();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
