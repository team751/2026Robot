package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class ExtendCommand extends Command {
    private final IntakeSubsystem intakeSubsystem = IntakeSubsystem.getInstance();

    public ExtendCommand() {
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.requestExtending();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.requestIdle();
        intakeSubsystem.requestRetracting();
    }
}
