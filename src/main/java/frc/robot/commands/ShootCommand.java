package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferSubsystem;

/**
 * Spins up the flywheel and feeds pieces into it via the transfer subsystem. Two modes:
 *
 * <ul>
 *   <li>Default (driver right trigger): distance-based speed — {@link
 *       ShooterSubsystem#requestShoot()} calculates flywheel RPS from how far the robot is from the
 *       hub.
 *   <li>{@code noDistance = true} (operator right trigger, "AASHOOT"): fires at a fixed speed
 *       regardless of distance — used when the operator wants a simple, predictable shot without
 *       relying on odometry/vision being accurate.
 * </ul>
 */
public class ShootCommand extends Command {
  private final ShooterSubsystem shooter;
  private final TransferSubsystem transfer;
  private Boolean noDistance = false;

  public ShootCommand(ShooterSubsystem shooter, TransferSubsystem transfer) {
    this.shooter = shooter;
    this.transfer = transfer;
    // addRequirements(shooter, transfer);
  }

  public ShootCommand(ShooterSubsystem shooter, TransferSubsystem transfer, Boolean noDistance) {
    this.shooter = shooter;
    this.transfer = transfer;
    this.noDistance = noDistance;
    // addRequirements(shooter, transfer);
  }

  @Override
  public void initialize() {
    if (noDistance) {
      shooter.requestAutoAimlessShoot();
    } else {
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
