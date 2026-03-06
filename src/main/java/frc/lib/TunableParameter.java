package frc.lib;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.function.DoubleConsumer;

public class TunableParameter {
  private static final ArrayList<TunableParameter> parameters = new ArrayList<>();

  private final String name;
  private double lastValue;
  private final DoubleConsumer callback;

  public TunableParameter(String name, double initialValue, DoubleConsumer callback) {
    this.name = name;
    this.lastValue = initialValue;
    this.callback = callback;

    SmartDashboard.putNumber(name, initialValue);

    parameters.add(this);
  }

  private void fetch() {
    double value = SmartDashboard.getNumber(name, lastValue);
    if (value != lastValue) {
      lastValue = value;
      callback.accept(value);
    }
  }

  public static void updateAll() {
    for (TunableParameter parameter : parameters) parameter.fetch();
  }
}
