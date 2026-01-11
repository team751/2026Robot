package frc.robot.subsystems.auton;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.drive.SwerveSubsystem;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

public class AutonSubsystem {
	private final AutoChooser autoChooser = new AutoChooser();

	private final AutoFactory autoFactory;

	private final Superstructure superstructure;
	private final SwerveSubsystem swerveSubsystem;

	private static AutonSubsystem instance;

	private AutonSubsystem() {
		superstructure = Superstructure.getInstance();
		swerveSubsystem = SwerveSubsystem.getInstance();

		autoFactory =
				new AutoFactory(
						swerveSubsystem::getPose,
						swerveSubsystem::resetPose,
						swerveSubsystem::followPath,
						false,
						swerveSubsystem);

		// autoChooser.addRoutine("hardcode auton", () -> getBadAuton());

		SmartDashboard.putData("Auto Chooser", autoChooser);
	}

	public static AutonSubsystem getInstance() {
		if (instance == null) instance = new AutonSubsystem();
		return instance;
	}

	public Command getSelectedAuton() {
		return autoChooser.selectedCommand();
	}

	private AutoRoutine getAuton(String name) {
		AutoRoutine routine = autoFactory.newRoutine(name);
		List<Command> commandList = new ArrayList<>();

		int index = 0;
		while (true) {
			AutoTrajectory trajectory = routine.trajectory(name, index);
			if (trajectory.getFinalPose().equals(Optional.empty())) break;
			if (index == 0) commandList.add(trajectory.resetOdometry());
			commandList.add(trajectory.cmd());

			// commandList.add(new AssistCommand(false, true));
			// commandList.add(new WaitCommand(1));
			index++;
		}

		// Register the full sequence of commands to run when routine is active
		routine.active().onTrue(Commands.sequence(commandList.toArray(new Command[0])));
		return routine;
	}

	private AutoRoutine getExampleAuton() {
		AutoRoutine routine = autoFactory.newRoutine("ExampleAuton");
		AutoTrajectory trajectory = routine.trajectory("ExampleAuton");

		routine.active().onTrue(trajectory.resetOdometry().andThen(trajectory.cmd()));
		return routine;
	}
}
