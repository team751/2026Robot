package frc.lib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import frc.robot.Robot;
import java.util.function.Supplier;

/**
 * A builder for CTRE devices (TalonFX, CANcoder, etc). Instead of writing out CAN ID, bus, and
 * TalonFXConfiguration setup by hand for every motor, each *Constants class builds one of these
 * with the chained {@code .withX()} methods and then calls {@link #createDevice} to get back a
 * real, configured device object.
 *
 * <p>Example: {@code CTREConfig<TalonFX, TalonFXConfiguration> config = new
 * CTREConfig<>(TalonFXConfiguration::new).withName("Flywheel").withCanID(33);}
 *
 * @param <Device> the CTRE hardware type being built, e.g. TalonFX
 * @param <Config> the matching configuration type, e.g. TalonFXConfiguration
 */
public class CTREConfig<Device extends ParentDevice, Config extends ParentConfiguration> {
  public String name = "UNNAMED";
  public int canID = 0;
  public CANBus canbus = Robot.riobus;
  public Config config;
  public boolean optimizeBus = true;

  public CTREConfig(Supplier<Config> configSupplier) {
    this.config = configSupplier.get();
  }

  public CTREConfig<Device, Config> withName(String name) {
    this.name = name;
    return this;
  }

  public CTREConfig<Device, Config> withCanID(int canID) {
    this.canID = canID;
    return this;
  }

  public CTREConfig<Device, Config> withBus(CANBus bus) {
    this.canbus = bus;
    return this;
  }

  public CTREConfig<Device, Config> withOptimizeBus(boolean optimizeBus) {
    this.optimizeBus = optimizeBus;
    return this;
  }

  /**
   * Actually constructs the hardware device (e.g. {@code new TalonFX(canID, canbus)}), applies the
   * {@link #config} to it, and — for TalonFX motors — trims the CAN bus traffic down to just the
   * signals we actually read (position, velocity, voltage, current) at 50Hz instead of the default
   * firehose. This keeps the CAN bus from getting congested as more devices are added.
   *
   * @param deviceSupplier a constructor reference like {@code TalonFX::new}
   */
  public Device createDevice(DeviceSupplier<Device> deviceSupplier) {
    Device device = deviceSupplier.get(canID, canbus);

    if (device == null || !device.isConnected()) {
      // System.err.println("Device " + name + " not connected (" + canID + " @ " + canbus + ")");
    }

    CTREUtil.applyConfiguration(device, config);

    if (optimizeBus && device instanceof TalonFX talon) {
      StatusSignal<Angle> positionSignal = talon.getPosition();
      StatusSignal<AngularVelocity> velocitySignal = talon.getVelocity();
      StatusSignal<Voltage> voltageSignal = talon.getMotorVoltage();
      StatusSignal<Current> currentStatorSignal = talon.getStatorCurrent();
      StatusSignal<Current> currentSupplySignal = talon.getSupplyCurrent();

      BaseStatusSignal[] signals =
          new BaseStatusSignal[] {
            positionSignal, velocitySignal, voltageSignal, currentStatorSignal, currentSupplySignal
          };

      // Only stream the signals we listed above, and drop everything else CTRE would
      // otherwise broadcast by default — this is what "optimizing" the bus means here.
      CTREUtil.tryUntilOK(
          () -> BaseStatusSignal.setUpdateFrequencyForAll(50.0, signals), talon.getDeviceID());
      CTREUtil.tryUntilOK(talon::optimizeBusUtilization, talon.getDeviceID());
    }
    return device;
  }

  @Override
  public String toString() {
    return name + ": " + canID + " @ " + canbus.getName();
  }

  public interface DeviceSupplier<Device> {
    Device get(int canID, CANBus canbus);
  }
}
