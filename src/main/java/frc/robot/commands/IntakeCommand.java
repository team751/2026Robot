package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.ExtenderSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

/**
 * Coordinates extending the intake and then spinning it once it's actually out. Extending and
 * running the intake motor at the same time would just grind the mechanism against whatever it's
 * unfolding past, so this waits for {@link ExtenderSubsystem#isExtended()} before intaking. Runs
 * continuously while held (driver left trigger); see {@link #isFinished}.
 */
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
    // Only start spinning the intake wheels once the extender has finished unfolding.
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
