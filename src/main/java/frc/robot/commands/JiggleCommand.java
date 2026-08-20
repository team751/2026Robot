package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.ExtenderSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.transfer.TransferSubsystem;

/**
 * Oscillates the extender in and out (extend → jiggle in → retract → extend → ...) while running
 * the intake/transfer in sync, to shake loose a game piece that's jammed or stuck in the mechanism.
 * Bound to the cross button on both controllers, and also run alongside {@code ShootCommand} on the
 * driver's right trigger to help feed pieces into the shooter.
 *
 * <p>Note: {@code addRequirements()} is commented out, so this command does NOT claim the
 * intake/extender/transfer subsystems — it can run at the same time as other commands using them
 * (e.g. ShootCommand), which is intentional for the right-trigger shoot+jiggle combo, but means
 * this command won't get auto-cancelled if something else takes over those subsystems.
 */
public class JiggleCommand extends Command {
  private final IntakeSubsystem intake;
  private final ExtenderSubsystem extender;
  private final TransferSubsystem transfer;

  public JiggleCommand(
      IntakeSubsystem intake, ExtenderSubsystem extender, TransferSubsystem transfer) {
    this.intake = intake;
    this.extender = extender;
    this.transfer = transfer;
    // addRequirements(intake, extender, transfer);
  }

  @Override
  public void initialize() {
    extender.requestExtend();
    // intake.requestIntake();
    // shooter.requestSpit();
    transfer.requestReverse();
  }

  @Override
  public void execute() {
    // This is the actual "jiggle": once fully extended, immediately start retracting a bit
    // (JIGGLE_IN, see ExtenderSubsystem) while running intake+transfer forward. Once it
    // reaches fully retracted, go back to extending. Repeating this rapidly is what shakes
    // a stuck piece free.
    if (extender.isExtended()) {
      extender.requestJiggleIn();
      transfer.requestTransfer();
      intake.requestIntake();
      // shooter.requestShoot();
    } else if (extender.isRetracted()) {
      extender.requestExtend();
      transfer.requestReverse();
      intake.requestIdle();
      // shooter.requestSpit();
    }
  }

  @Override
  public void end(boolean interrupted) {
    intake.requestIdle();
    extender.requestRetract();
    // shooter.requestIdle();
    transfer.requestIdle();
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
