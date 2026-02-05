package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeCommand extends Command {
private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

public IntakeCommand() {
	addRequirements(intakeSubsystem);
}

@Override
public void initialize() {
	intakeSubsystem.requestIntaking();
    intakeSubsystem.requestExtending();
    intakeSubsystem.requestRetracting();
    intakeSubsystem.requestSpitting();
}
@Override
public void end(boolean interrupted) {
    intakeSubsystem.requestIdle();
}
}