package frc.lib;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class NTDouble<U extends Unit> implements DoubleSupplier, Supplier<Measure<U>> {

  private final DoubleSubscriber subscriber;

  private final U Unit;

  public NTDouble(String name, U Unit, Measure<U> defaultValue) {
    this.Unit = Unit;
    this.subscriber = getSubscriber(createTopic(name), defaultValue);
  }

  public NTDouble(String name, U Unit) {
    // TODO Unchecked cast warning; pretty sure this is safe
    this(name, Unit, (Measure<U>) Unit.of(0.0));
  }

  private DoubleTopic createTopic(String name) {
    String nameWithUnit = name + " (" + Unit.name() + ")";
    return NetworkTableInstance.getDefault().getDoubleTopic(nameWithUnit);
  }

  private DoubleSubscriber getSubscriber(DoubleTopic topic, Measure<U> defaultValue) {
    double doubleValue = defaultValue.in(Unit);
    topic.publish().set(doubleValue);
    return topic.subscribe(doubleValue);
  }

  @Override
  public double getAsDouble() {
    return subscriber.get();
  }

  @Override
  public Measure<U> get() {
    // TODO Use MutableMeasure to avoid additional allocations
    // TODO Unchecked cast warning; pretty sure this is safe
    return (Measure<U>) Unit.of(getAsDouble());
  }
}
