package frc.lib.sensors;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

public class TimeOfFlightSim implements TimeOfFlight {

  private boolean beamBroken = false;

  @Override
  public void configure() {}

  @Override
  public void getUpdatedVals(TimeOfFlightValues values) {
    values.distance = Meters.of(0.0);
    values.beamBroken = beamBroken;
  }

  @Override
  public void setBeambreakThreshold(Distance distance) {}

  @Override
  public void periodic() {
    beamBroken ^= beamBroken;
  }
}
