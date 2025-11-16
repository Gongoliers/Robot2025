package frc.lib.sensors;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;

/**
 * Basic interface for time of flight sensors
 *
 * <p>Outlines required functions and variables all time of flight sensors have to make implementing
 * new types of time of flight sensors in the future much simpler
 */
public interface TimeOfFlight {

  /** Time of Flight values */
  public static class TimeOfFlightValues {

    /** Distance */
    public Distance distance = Meters.of(0.0);

    /** True if distance is below the beam broken threshhold */
    public boolean beamBroken = false;
  }

  /** Configures the time of flight sensor */
  public void configure();

  /**
   * Get the time of flight sensor's updated values and update the provided values class accordingly
   *
   * @param values values class to be updated
   */
  public void getUpdatedVals(TimeOfFlightValues values);

  /**
   * Set the distance threshold for beam to be considered broken
   *
   * @param distanceThreshold distance threshold
   */
  public void setBeambreakThreshold(Distance distanceThreshold);

  /** Called every periodic loop */
  public void periodic();
}
