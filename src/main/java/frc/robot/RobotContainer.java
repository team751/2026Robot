// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.JiggleCommand;
import frc.robot.subsystems.drive.SwerveSubsystem;
import frc.robot.subsystems.intake.ExtenderSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.transfer.TransferSubsystem;
import java.io.File;
import java.util.Arrays;

public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  private final AutoFactory autoFactory;

  public RobotContainer() {
    SwerveSubsystem swerve = SwerveSubsystem.getInstance();

    // Choreo trajectory follower. Alliance flipping is off to match the PathPlanner setup:
    // red autos get their own trajectories instead of being flipped at runtime.
    autoFactory =
        new AutoFactory(
            swerve::getPose,
            swerve::resetPose, // only runs if a routine explicitly resets odometry
            swerve::followTrajectory, // called every loop with the current SwerveSample
            false, // useAllianceFlipping
            swerve);

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
    registerEvent("Shoot", shoot);
    registerEvent("StopShoot", stopShoot);

    /*Jiggle */
    registerEvent("Jiggle", jiggle);
    registerEvent("StopJiggle", jigglestop);

    /*Intake */
    registerEvent("Intake", intake);
    registerEvent("StopIntake", stopIntake);
    registerEvent("Spit", spit);
    registerEvent("StopSpit", stopSpit);
    /*Extender */
    registerEvent("Retract", retract);
    /*Transfer */
    registerEvent("Transfer", transfer);
    registerEvent("StopTransfer", stopTransfer);

    autoChooser = AutoBuilder.buildAutoChooser();
    addChoreoTrajectories();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  /** Registers a command as both a PathPlanner named command and a Choreo event marker. */
  private void registerEvent(String name, Command command) {
    NamedCommands.registerCommand(name, command);
    autoFactory.bind(name, command);
  }

  /**
   * Adds every trajectory in deploy/choreo/ to the chooser as "Choreo: name". Each one runs in an
   * AutoRoutine (not trajectoryCmd) so event markers fire the commands bound in registerEvent.
   * Odometry is reset to the trajectory start in simulation only; on the robot, vision sets it.
   */
  private void addChoreoTrajectories() {
    File[] trajFiles =
        new File(Filesystem.getDeployDirectory(), "choreo")
            .listFiles((dir, fileName) -> fileName.endsWith(".traj"));
    if (trajFiles == null) return; // no choreo folder yet

    Arrays.sort(trajFiles);
    for (File file : trajFiles) {
      String name = file.getName().replace(".traj", "");
      AutoRoutine routine = autoFactory.newRoutine(name);
      AutoTrajectory trajectory = routine.trajectory(name);
      routine
          .active()
          .onTrue(
              RobotBase.isSimulation()
                  ? trajectory.resetOdometry().andThen(trajectory.cmd())
                  : trajectory.cmd());
      autoChooser.addOption("Choreo: " + name, routine.cmd());
    }
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
