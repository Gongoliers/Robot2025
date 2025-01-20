package frc.lib.controllers.position;

/** 
 * Basic interface for linear position controllers
 * 
 * Outlines required functions and variables all linear position controllers have to make implementing new types
 * of position controllers in the future much simpler
 */
public interface LinearPositionController {
  
  /** Linear position controller values */
  public static class LinearPositionControllerValues {

    /** Current position in inches */
    public double posInches = 0.0;

    /** Current velocity in inches per second */
    public double velInchesPerSec = 0.0;

    /** Current acceleration in inches per second per second */
    public double accInchesPerSecPerSec = 0.0;

    /** Current motor voltage */
    public double motorVolts = 0.0;

    /** Current motor current */
    public double motorAmps = 0.0;
  }

  /** Configures the linear position controller */
  public void configure();

  /**
   * Get the linear position controller's updated values and update the provided values class accordingly
   * 
   * @param values values class to be updated
   */
  public void getUpdatedVals(LinearPositionControllerValues values);

  /**
   * Set current position of linear position controller
   * 
   * @param posInches position in inches
   */
  public void setPos(double posInches);

  /**
   * Set setpoint of linear position controller
   * 
   * @param posInches setpoint position in inches
   * @param velInchesPerSec setpoint velocity in inches per second
   */
  public void setSetpoint(double posInches, double velInchesPerSec);

  /** Called every periodic loop */
  public void periodic();
}
