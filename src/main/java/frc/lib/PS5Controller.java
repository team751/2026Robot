package frc.lib;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller.Axis;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

/**
 * PS5 DualSense wrapper that works both when the Driver Station sees the controller directly and
 * when DS4Windows presents it as an Xbox 360 (XInput) controller. The Driver Station can only
 * rumble XInput controllers, so DS4Windows is needed for haptics, but it renumbers every button
 * and axis. The mapping is picked on every read from what the Driver Station currently reports.
 */
public class PS5Controller {
  /** How far an XInput trigger must be pulled to count as pressed (WPILib's default). */
  private static final double XINPUT_TRIGGER_THRESHOLD = 0.5;

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
  public final Trigger leftJoystickButton;
  public final Trigger rightJoystickButton;

  /* Bumpers */
  public final Trigger leftBumper;
  public final Trigger rightBumper;

  /* Buttons */
  public final Trigger triangleButton;
  public final Trigger circleButton;
  public final Trigger squareButton;
  public final Trigger crossButton;

  /* D-Pad */
  public final Trigger dUp;
  public final Trigger dRight;
  public final Trigger dDown;
  public final Trigger dLeft;

  /* Touchpad (not available through XInput) */
  public final Trigger touchpadButton;

  public PS5Controller(int port) {
    joystick = new Joystick(port);

    leftVerticalJoystick = () -> -axis(Axis.kLeftY.value, XboxController.Axis.kLeftY.value);
    leftHorizontalJoystick = () -> -axis(Axis.kLeftX.value, XboxController.Axis.kLeftX.value);
    rightHorizontalJoystick = () -> -axis(Axis.kRightX.value, XboxController.Axis.kRightX.value);
    rightVerticalJoystick = () -> -axis(Axis.kRightY.value, XboxController.Axis.kRightY.value);

    leftTrigger = trigger(Button.kL2.value, XboxController.Axis.kLeftTrigger.value);
    rightTrigger = trigger(Button.kR2.value, XboxController.Axis.kRightTrigger.value);

    leftBumper = button(Button.kL1.value, XboxController.Button.kLeftBumper.value);
    rightBumper = button(Button.kR1.value, XboxController.Button.kRightBumper.value);

    leftJoystickButton = button(Button.kL3.value, XboxController.Button.kLeftStick.value);
    rightJoystickButton = button(Button.kR3.value, XboxController.Button.kRightStick.value);

    triangleButton = button(Button.kTriangle.value, XboxController.Button.kY.value);
    circleButton = button(Button.kCircle.value, XboxController.Button.kB.value);
    squareButton = button(Button.kSquare.value, XboxController.Button.kX.value);
    crossButton = button(Button.kCross.value, XboxController.Button.kA.value);

    dUp = new Trigger(() -> joystick.getPOV() == 0.0);
    dRight = new Trigger(() -> joystick.getPOV() == 90.0);
    dDown = new Trigger(() -> joystick.getPOV() == 180.0);
    dLeft = new Trigger(() -> joystick.getPOV() == 270.0);

    touchpadButton =
        new Trigger(() -> !isXInput() && joystick.getRawButton(Button.kTouchpad.value));
  }

  /** True when the Driver Station sees this controller as XInput (e.g. through DS4Windows). */
  public boolean isXInput() {
    return DriverStation.getJoystickIsXbox(joystick.getPort());
  }

  private double axis(int ps5Axis, int xinputAxis) {
    return joystick.getRawAxis(isXInput() ? xinputAxis : ps5Axis);
  }

  private Trigger button(int ps5Button, int xinputButton) {
    return new Trigger(() -> joystick.getRawButton(isXInput() ? xinputButton : ps5Button));
  }

  /** PS5 reports L2/R2 as buttons; XInput only reports them as 0 to 1 axes. */
  private Trigger trigger(int ps5Button, int xinputAxis) {
    return new Trigger(
        () ->
            isXInput()
                ? joystick.getRawAxis(xinputAxis) > XINPUT_TRIGGER_THRESHOLD
                : joystick.getRawButton(ps5Button));
  }

  /**
   * Rumbles the controller while the returned command is running, then stops it on end. Only works
   * when the controller is seen as XInput (see {@link #isXInput()}).
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
