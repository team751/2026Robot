// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.JiggleCommand;
import frc.robot.subsystems.intake.ExtenderSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    // COMMANDS FOR NAMED COMMANDS
    Command spit =
        new RunCommand(
            () -> IntakeSubsystem.getInstance().requestSpit(), IntakeSubsystem.getInstance());
    InstantCommand stopSpit =
        new InstantCommand(
            () -> {
              spit.cancel();
              IntakeSubsystem.getInstance().requestIdle();
            });

    Command jiggle =
        new JiggleCommand(
            IntakeSubsystem.getInstance(),
            ExtenderSubsystem.getInstance(),
            TransferSubsystem.getInstance());

    Command jigglestop = new InstantCommand(() -> jiggle.cancel());

    Command intake =
        new SequentialCommandGroup(
            new RunCommand(
                    () -> ExtenderSubsystem.getInstance().requestExtend(),
                    ExtenderSubsystem.getInstance())
                .until(() -> ExtenderSubsystem.getInstance().isExtended()),
            new RunCommand(
                () -> IntakeSubsystem.getInstance().requestIntake(),
                IntakeSubsystem.getInstance()));
    InstantCommand stopIntake =
        new InstantCommand(
            () -> {
              intake.cancel();
              IntakeSubsystem.getInstance().requestIdle();
            });

    Command transfer =
        new RunCommand(
            () -> TransferSubsystem.getInstance().requestTransfer(),
            TransferSubsystem.getInstance());
    InstantCommand stopTransfer =
        new InstantCommand(
            () -> {
              transfer.cancel();
              TransferSubsystem.getInstance().requestIdle();
            });

    Command shoot =
        new ParallelCommandGroup(
            new RunCommand(
                () -> ShooterSubsystem.getInstance().requestShoot(),
                ShooterSubsystem.getInstance()),
            new RunCommand(
                () -> TransferSubsystem.getInstance().requestTransfer(),
                TransferSubsystem.getInstance()));
    InstantCommand stopShoot =
        new InstantCommand(
            () -> {
              shoot.cancel();
              ShooterSubsystem.getInstance().requestIdle();
              TransferSubsystem.getInstance().requestIdle();
            });

    Command retract =
        new RunCommand(
                () -> ExtenderSubsystem.getInstance().requestRetract(),
                ExtenderSubsystem.getInstance())
            .until(() -> ExtenderSubsystem.getInstance().isRetracted());

    /*Shooter */
    NamedCommands.registerCommand("Shoot", shoot);
    NamedCommands.registerCommand("StopShoot", stopShoot);

    /*Jiggle */
    NamedCommands.registerCommand("Jiggle", jiggle);
    NamedCommands.registerCommand("StopJiggle", jigglestop);

    /*Intake */
    NamedCommands.registerCommand("Intake", intake);
    NamedCommands.registerCommand("StopIntake", stopIntake);
    NamedCommands.registerCommand("Spit", spit);
    NamedCommands.registerCommand("StopSpit", stopSpit);
    /*Extender */
    NamedCommands.registerCommand("Retract", retract);
    /*Transfer */
    NamedCommands.registerCommand("Transfer", transfer);
    NamedCommands.registerCommand("StopTransfer", stopTransfer);

    // LoggedDashboardChooser wraps the normal SendableChooser: it still shows up on the
    // dashboard exactly the same, but the selected auto name is now recorded to the log too.
    autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}