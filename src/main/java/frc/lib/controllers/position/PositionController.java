package frc.lib.controllers.position;

/** 
 * Basic interface for position controllers
 * 
 * Outlines required functions and variables all position controllers have to make implementing new types
 * of position controllers in the future much simpler
 */
public interface PositionController {
  
  /** Position controller values */
  public static class PositionControllerValues {
    
    /** Current position in rotations */
    public double posRotations = 0.0;

    /** Current velocity in rotations per second */
    public double velRotationsPerSec = 0.0;

    /** Current acceleration in rotations per second per second */
    public double accRotationsPerSecPerSec = 0.0;

    /** Current voltage */
    public double motorVolts = 0.0;

    /** Current current */
    public double motorAmps = 0.0;
  }

  /** Configures the position controller */
  public void configure();

  /**
   * Get the position controller's updated values and update the provided values class accordingly
   * 
   * @param values values class to be updated
   */
  public void getUpdatedVals(PositionControllerValues values);

  /**
   * Sets the position of the position controller
   * 
   * @param posRotations position in rotations
   */
  public void setPos(double posRotations);

  /**
   * Sets the setpoint (target position and velocity) of the controller
   * 
   * @param posRotations target position in rotations
   * @param velRotationsPerSec target velocity in rotations per second
   */
  public void setSetpoint(double posRotations, double velRotationsPerSec);
}
