package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IdleCommand extends Command {
private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

public IdleCommand() {
	addRequirements(intakeSubsystem);
}

@Override
public void initialize() {
	intakeSubsystem.requestIdle();
}
@Override
public void end(boolean interrupted) {
    intakeSubsystem.requestIntaking();
    intakeSubsystem.requestSpitting();
	intakeSubsystem.requestExtending();
	intakeSubsystem.requestRetracting();
}
}