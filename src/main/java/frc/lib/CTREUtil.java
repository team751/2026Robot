package frc.lib;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.*;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/**
 * Small helpers for talking to CTRE (Cross The Road Electronics) devices reliably. CAN bus calls
 * can occasionally fail/time out for no real reason, so instead of trusting the first attempt, we
 * retry a few times and only complain to the driver station if it never succeeds.
 */
public class CTREUtil {
  /**
   * Calls {@code function} up to 10 times until it returns {@link StatusCode#OK}. Used for every
   * CTRE call in this class so a single dropped CAN frame doesn't silently leave a motor
   * unconfigured.
   */
  public static StatusCode tryUntilOK(Supplier<StatusCode> function, int deviceId) {
    final int max_num_retries = 10;
    StatusCode statusCode = StatusCode.OK;
    for (int i = 0; i < max_num_retries; ++i) {
      statusCode = function.get();
      if (statusCode == StatusCode.OK) break;
    }
    if (statusCode != StatusCode.OK) {
      DriverStation.reportError(
          "Error calling " + function + " on ctre device id " + deviceId + ": " + statusCode, true);
    }
    return statusCode;
  }

  /** Pushes a full config object onto a device, retrying via {@link #tryUntilOK}. */
  public static StatusCode applyConfiguration(TalonFX motor, TalonFXConfiguration config) {
    return tryUntilOK(() -> motor.getConfigurator().apply(config), motor.getDeviceID());
  }

  public static StatusCode applyConfiguration(CANrange sensor, CANrangeConfiguration config) {
    return tryUntilOK(() -> sensor.getConfigurator().apply(config), sensor.getDeviceID());
  }

  public static StatusCode applyConfiguration(CANcoder cancoder, CANcoderConfiguration config) {
    return tryUntilOK(() -> cancoder.getConfigurator().apply(config), cancoder.getDeviceID());
  }

  public static StatusCode applyConfiguration(CANdle candle, CANdleConfiguration config) {
    return tryUntilOK(() -> candle.getConfigurator().apply(config), candle.getDeviceID());
  }

  /**
   * Same as the type-specific overloads above, but dispatches on the device's runtime type. This is
   * what {@link CTREConfig#createDevice} calls, since it's generic over device type and doesn't
   * know at compile time which overload applies.
   */
  public static StatusCode applyConfiguration(ParentDevice device, ParentConfiguration config) {
    if (device instanceof TalonFX)
      return applyConfiguration((TalonFX) device, (TalonFXConfiguration) config);
    else if (device instanceof CANcoder)
      return applyConfiguration((CANcoder) device, (CANcoderConfiguration) config);
    else if (device instanceof CANrange)
      return applyConfiguration((CANrange) device, (CANrangeConfiguration) config);
    else if (device instanceof CANdle)
      return applyConfiguration((CANdle) device, (CANdleConfiguration) config);
    throw new IllegalArgumentException("Device type not supported");
  }

  public static StatusCode refreshConfiguration(TalonFX motor, TalonFXConfiguration config) {
    return tryUntilOK(() -> motor.getConfigurator().refresh(config), motor.getDeviceID());
  }
}
