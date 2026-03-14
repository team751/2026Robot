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
import frc.robot.subsystems.intake.ExtenderSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferSubsystem;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // COMMANDS FOR NAMED COMMANDS
    Command spit =
        new RunCommand(
            () -> IntakeSubsystem.getInstance().requestSpitting(), IntakeSubsystem.getInstance());
    InstantCommand stopSpit =
        new InstantCommand(
            () -> {
              spit.cancel();
              IntakeSubsystem.getInstance().requestIdle();
            });

    Command intake =
        new SequentialCommandGroup(
            new RunCommand(
                    () -> ExtenderSubsystem.getInstance().requestExtending(),
                    ExtenderSubsystem.getInstance())
                .until(() -> ExtenderSubsystem.getInstance().isAtExtendLimit()),
            new RunCommand(
                () -> IntakeSubsystem.getInstance().requestIntaking(),
                IntakeSubsystem.getInstance()));
    InstantCommand stopIntake =
        new InstantCommand(
            () -> {
              intake.cancel();
              IntakeSubsystem.getInstance().requestIdle();
              ExtenderSubsystem.getInstance().requestIdle();
            });

    Command transfer =
        new RunCommand(
            () -> TransferSubsystem.getInstance().requestTransferring(),
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
                () -> TransferSubsystem.getInstance().requestTransferring(),
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
                () -> ExtenderSubsystem.getInstance().requestRetracting(),
                ExtenderSubsystem.getInstance())
            .until(() -> ExtenderSubsystem.getInstance().isAtRetractLimit());
    InstantCommand stopRetract =
        new InstantCommand(
            () -> {
              retract.cancel();
              ExtenderSubsystem.getInstance().requestIdle();
            });

    /*Shooter */
    NamedCommands.registerCommand("Shoot", shoot);
    NamedCommands.registerCommand("StopShoot", stopShoot);
    /*Intake */
    NamedCommands.registerCommand("Intake", intake);
    NamedCommands.registerCommand("StopIntake", stopIntake);
    NamedCommands.registerCommand("Spit", spit);
    NamedCommands.registerCommand("StopSpit", stopSpit);
    /*Extender */
    NamedCommands.registerCommand("Retract", retract);
    NamedCommands.registerCommand("StopRetract", stopRetract);
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
