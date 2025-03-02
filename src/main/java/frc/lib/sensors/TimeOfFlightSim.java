package frc.lib.sensors;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TimeOfFlightSim implements TimeOfFlight {

  private boolean beamBroken = false;

  @Override
  public void configure() {
    
  }

  @Override
  public void getUpdatedVals(TimeOfFlightValues values) {
    values.distanceMeters = 0.0;
    values.beamBroken = true;
  }

  @Override
  public void setBeambreakThreshold(double distanceMeters) {
    
  }

  @Override
  public Trigger beamBroken() {
    return new Trigger(() -> beamBroken);
  }

  @Override
  public void periodic() {
    beamBroken ^= beamBroken;
  }
}
