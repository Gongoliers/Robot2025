package frc.lib.sensors;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.units.measure.Distance;
import frc.lib.CAN;
import frc.lib.configs.appliers.CANrangeConfigApplier;

public class TimeOfFlightCANrange implements TimeOfFlight {

  private CANrange timeOfFlight;

  private final StatusSignal<Distance> distance;

  private Distance beambreakThreshold;

  public TimeOfFlightCANrange(CAN timeOfFlightCAN) {

    timeOfFlight = new CANrange(timeOfFlightCAN.id(), timeOfFlightCAN.bus());

    distance = timeOfFlight.getDistance();
  }

  @Override
  public void configure() {
    BaseStatusSignal.setUpdateFrequencyForAll(100, distance);

    CANrangeConfigApplier.applyFactoryDefault(timeOfFlight);
  }

  @Override
  public void getUpdatedVals(TimeOfFlightValues values) {
    BaseStatusSignal.refreshAll(distance);
    values.distance = Meters.of(distance.getValueAsDouble());
    values.beamBroken =
        values.distance.baseUnitMagnitude() < beambreakThreshold.baseUnitMagnitude();
  }

  @Override
  public void setBeambreakThreshold(Distance distanceMeters) {
    beambreakThreshold = distanceMeters;
  }

  @Override
  public void periodic() {}
}
