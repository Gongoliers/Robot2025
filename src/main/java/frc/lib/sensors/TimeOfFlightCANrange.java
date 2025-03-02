package frc.lib.sensors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.CAN;
import frc.lib.configs.appliers.CANrangeConfigApplier;

public class TimeOfFlightCANrange implements TimeOfFlight {
  
  private CANrange timeOfFlight;

  private final StatusSignal<Distance> distance;

  private double beambreakThresholdMeters;

  public TimeOfFlightCANrange(
      CAN timeOfFlightCAN) {
    
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
    values.distanceMeters = distance.getValueAsDouble();
    values.beamBroken = values.distanceMeters < beambreakThresholdMeters;
  }

  @Override
  public void setBeambreakThreshold(double distanceMeters) {
    beambreakThresholdMeters = distanceMeters;
  }

  @Override
  public Trigger beamBroken() {
    return new Trigger(() -> distance.getValueAsDouble() < beambreakThresholdMeters);
  }

  @Override
  public void periodic() {

  }
}
