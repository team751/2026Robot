package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.lib.PS5Controller;
//import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.climber.ClimberSubsystem;

public class ControlBoard {
	private static ControlBoard instance;

	/* Controllers */
	private PS5Controller driver = null;
	private PS5Controller operator = null;
	private ClimberSubsystem climber = ClimberSubsystem.getInstance();

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
		// controller.triangleButton.whileTrue(
		// 	new StartEndCommand(() -> /* COMMAND ON START */, () -> /* COMMAND ON END */)
		// 	.withName("Shooter Shooty")
		// );
		// like lwk just kinda copy that lol, but in the COMMAND ON START and COMMAND ON END sections, put a method call
		// ex:
		// (in ClimberSubsystem.java)
		// public static void moveUp() {
		// 		motor.move();
		// }
		//
		// (in ControlBoard.java)
		// ...
		// new StartEndCommand(() -> climber.moveUp(), () -> climber.stopMotors())
		// ...

		controller.leftTrigger.whileTrue(
			new StartEndCommand(() -> climber.moveUp180(), () -> climber.stopMotors();)
		);

		controller.rightTrigger.whileTrue(
			new StartEndCommand(() -> climber.moveDown180(), () -> climber.stopMotors())
		);
		
	}

	private void configureOperatorBindings(PS5Controller controller) {}
}
