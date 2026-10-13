package frc.lib;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

/**
 * PS5 controller running through DS4Windows as an emulated Xbox 360 controller (so the Driver
 * Station can send rumble). Fields keep PlayStation names for the physical buttons, but read the
 * Xbox (XInput) indices that the Driver Station actually sees.
 */
public class PS5Controller {
  /* How far a trigger has to be pressed to count as held */
  private static final double TRIGGER_THRESHOLD = 0.5;

  /* Axes */
  public final DoubleSupplier leftVerticalJoystick;
  public final DoubleSupplier leftHorizontalJoystick;
  public final DoubleSupplier rightHorizontalJoystick;
  public final DoubleSupplier rightVerticalJoystick;

  /* Controllers */
  public final Joystick joystick;

  /* Triggers */
  public final Trigger leftTrigger;
  public final Trigger rightTrigger;

  /* Joystick Buttons */
  public final JoystickButton leftJoystickButton;
  public final JoystickButton rightJoystickButton;

  /* Bumpers */
  public final JoystickButton leftBumper;
  public final JoystickButton rightBumper;

  /* Buttons */
  public final JoystickButton triangleButton;
  public final JoystickButton circleButton;
  public final JoystickButton squareButton;
  public final JoystickButton crossButton;

  /* D-Pad */
  public final Trigger dUp;
  public final Trigger dRight;
  public final Trigger dDown;
  public final Trigger dLeft;

  /* Center buttons (DS4Windows: Create -> Back, Options -> Start) */
  public final JoystickButton createButton;
  public final JoystickButton optionsButton;

  public PS5Controller(int port) {
    joystick = new Joystick(port);

    leftVerticalJoystick = () -> -joystick.getRawAxis(Axis.kLeftY.value);
    leftHorizontalJoystick = () -> -joystick.getRawAxis(Axis.kLeftX.value);
    rightHorizontalJoystick = () -> -joystick.getRawAxis(Axis.kRightX.value);
    rightVerticalJoystick = () -> -joystick.getRawAxis(Axis.kRightY.value);

    // Xbox triggers are analog axes (0 to 1), not buttons
    leftTrigger =
        new Trigger(() -> joystick.getRawAxis(Axis.kLeftTrigger.value) > TRIGGER_THRESHOLD);
    rightTrigger =
        new Trigger(() -> joystick.getRawAxis(Axis.kRightTrigger.value) > TRIGGER_THRESHOLD);

    leftBumper = new JoystickButton(joystick, Button.kLeftBumper.value);
    rightBumper = new JoystickButton(joystick, Button.kRightBumper.value);

    leftJoystickButton = new JoystickButton(joystick, Button.kLeftStick.value);
    rightJoystickButton = new JoystickButton(joystick, Button.kRightStick.value);

    // DS4Windows maps by position: Cross -> A, Circle -> B, Square -> X, Triangle -> Y
    triangleButton = new JoystickButton(joystick, Button.kY.value);
    circleButton = new JoystickButton(joystick, Button.kB.value);
    squareButton = new JoystickButton(joystick, Button.kX.value);
    crossButton = new JoystickButton(joystick, Button.kA.value);

    dUp = new Trigger(() -> joystick.getPOV() == 0.0);
    dRight = new Trigger(() -> joystick.getPOV() == 90.0);
    dDown = new Trigger(() -> joystick.getPOV() == 180.0);
    dLeft = new Trigger(() -> joystick.getPOV() == 270.0);

    createButton = new JoystickButton(joystick, Button.kBack.value);
    optionsButton = new JoystickButton(joystick, Button.kStart.value);
  }

  /**
   * Rumbles the controller while the returned command is running, then stops it on end.
   *
   * @param strength 0.0 to 1.0
   */
  public Command rumble(double strength) {
    return rumble(() -> strength);
  }

  /**
   * Rumbles the controller while the returned command is running, then stops it on end. The
   * strength is read when the command starts, not when it is created.
   *
   * @param strength 0.0 to 1.0
   */
  public Command rumble(DoubleSupplier strength) {
    return Commands.startEnd(
            () -> joystick.setRumble(RumbleType.kBothRumble, strength.getAsDouble()),
            () -> joystick.setRumble(RumbleType.kBothRumble, 0.0))
        .ignoringDisable(true)
        .withName("Rumble");
  }

  public double getRightTrigger() {
    return 0.0;
  }
}
