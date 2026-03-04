package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.lib.PS5Controller;
//import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ControlBoard {
	private static ControlBoard instance;

	/* Controllers */
	private PS5Controller driver = null;
	private PS5Controller operator = null;
	private ShooterSubsystem shooter = ShooterSubsystem.getInstance();

	private enum ControllerPreset {
		DRIVER(0),
		OPERATOR(1);

		private final int port;

		ControllerPreset(int port) {
			this.port = port;
		}

		public int port() {
			return port;
		}
	}


	private ControlBoard() {
		DriverStation.silenceJoystickConnectionWarning(true);

		tryInit();
	}

	public void tryInit() {
		if (driver == null) {
			driver = new PS5Controller(ControllerPreset.DRIVER.port());
			configureBindings(ControllerPreset.DRIVER, driver);
			System.out.println("Driver Initialized");
		}

		if (operator == null) {
			operator = new PS5Controller(ControllerPreset.OPERATOR.port());
			configureBindings(ControllerPreset.OPERATOR, operator);
			System.out.println("Operator Initialized");
		}
	}

	public void displayUI() {}

	public static ControlBoard getInstance() {
		if (instance == null) instance = new ControlBoard();
		return instance;
	}

	private void configureBindings(ControllerPreset preset, PS5Controller controller) {
		switch (preset) {
			case DRIVER -> configureDriverBindings(controller);
			case OPERATOR -> configureOperatorBindings(controller);
			default -> throw new IllegalStateException("Unexpected value: " + preset);
		}
	}

	private void configureDriverBindings(PS5Controller controller) {
		controller.triangleButton.whileTrue(
			new StartEndCommand(() -> shooter.newShooterSpeed(1,1), () -> shooter.requestIdle())
		);

		controller.dLeft.whileTrue(
			new StartEndCommand(() -> shooter.newShooterSpeed(2,2), () -> shooter.requestIdle())
		);

		controller.dUp.whileTrue(
			new StartEndCommand(() -> shooter.newShooterSpeed(5,5), () -> shooter.requestIdle())
		);
        
		controller.dRight.whileTrue(
			new StartEndCommand(() -> shooter.newShooterSpeed(7,7), () -> shooter.requestIdle())
		);

		controller.leftBumper.whileTrue(
			new StartEndCommand(() -> shooter.newShooterSpeed(2, 0), () -> shooter.requestIdle())
		);

		controller.rightBumper.whileTrue(
			new StartEndCommand(() -> shooter.newShooterSpeed(0, 2), () -> shooter.requestIdle())
		);

		// controller.rightTrigger.whileTrue(
		// 	new StartEndCommand(() -> shooter.newShooterSpeed(controller.getRightTrigger()), () -> shooter.requestIdle())
		// );

		// controller.rightTrigger.whileTrue(
		// 	new StartEndCommand(() -> System.out.println(controller.getRightTrigger()), () -> System.out.println("Stopped"))
		// );

		
	}

	private void configureOperatorBindings(PS5Controller controller) {}
}
