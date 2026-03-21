package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.intake.ExtenderSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferSubsystem;

public class JiggleCommand extends Command {
  private final IntakeSubsystem intake;
  private final ExtenderSubsystem extender;
  private final TransferSubsystem transfer;
  private final ShooterSubsystem shooter;

  public JiggleCommand(
      IntakeSubsystem intake,
      ExtenderSubsystem extender,
      TransferSubsystem transfer,
      ShooterSubsystem shooter) {
    this.intake = intake;
    this.extender = extender;
    this.transfer = transfer;
    this.shooter = shooter;
    addRequirements(intake, extender, transfer, shooter);
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
