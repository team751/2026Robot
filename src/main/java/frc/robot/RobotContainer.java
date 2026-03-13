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
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferSubsystem;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    // COMMANDS FOR NAMED COMMANDS
    // TODO: Make the commands end auton
    StartEndCommand shoot =
        new StartEndCommand(
            () -> ShooterSubsystem.getInstance().requestShoot(),
            () -> ShooterSubsystem.getInstance().requestIdle());
    InstantCommand stopShoot = new InstantCommand(() -> shoot.cancel());

    StartEndCommand intake =
        new StartEndCommand(
            () -> IntakeSubsystem.getInstance().requestIntaking(),
            () -> IntakeSubsystem.getInstance().requestIdle());
    InstantCommand stopIntake = new InstantCommand(() -> intake.cancel());

    StartEndCommand transfer =
        new StartEndCommand(
            () -> TransferSubsystem.getInstance().requestTransferring(),
            () -> TransferSubsystem.getInstance().requestIdle());
    InstantCommand stopTransfer = new InstantCommand(() -> transfer.cancel());

    /*Shooter */
    NamedCommands.registerCommand("shoot", shoot);
    NamedCommands.registerCommand("stopShoot", stopShoot);
    /*Intake */
    NamedCommands.registerCommand("intake", intake);
    NamedCommands.registerCommand("stopIntake", stopIntake);
    /*Transfer */
    NamedCommands.registerCommand("transfer", transfer);
    NamedCommands.registerCommand("stopTransfer", stopTransfer);

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
