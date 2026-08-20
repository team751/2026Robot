// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

/**
 * Registers "Named Commands" that PathPlanner autos can reference by string (e.g. an event marker
 * in a .path file saying "Shoot" runs the {@code shoot} command built below), and builds the auto
 * chooser dropdown shown on the dashboard. This only wires things up once at startup — actual
 * driver control bindings live in {@link frc.robot.util.ControlBoard}, not here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // Each of these blocks builds one Command and its "stop" counterpart, then registers
    // both under a name PathPlanner autos can reference. The stop commands cancel the
    // main command and force the subsystem back to idle, since a plain .cancel() alone
    // doesn't reliably leave motors at 0.
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

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
