package frc.lib.sensors;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Basic interface for time of flight sensors
 * 
 * Outlines required functions and variables all time of flight sensors have to make implementing new types
 * of time of flight sensors in the future much simpler
 */
public interface TimeOfFlight {
  
  /** Time of Flight values */
  public static class TimeOfFlightValues {

    /** Distance in meters */
    public double distanceMeters = 0.0;

    /** True if distance in meters is below the beam broken threshhold */
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
   * @param distanceMeters distance threshold in meters
   */
  public void setBeambreakThreshold(double distanceMeters);

  /**
   * Returns a trigger that is triggered when the distance is less than the beambreak threshold
   * 
   * @return a trigger that is triggered when the distance is less than the beambreak threshold
   */
  public Trigger beamBroken();

  /** Called every periodic loop */
  public void periodic();
}
