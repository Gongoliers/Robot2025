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

    /** Current motor position in rotations */
    public double motorPosRotations = 0.0;

    /** Current motor velocity in rotatoins per second */
    public double motorVelRotationsPerSec = 0.0;

    /** Current motor acceleration in rotations per second per second */
    public double motorAccRotationsPerSecPerSec = 0.0;

    /** Current motor voltage */
    public double motorVolts = 0.0;

    /** Current motor current */
    public double motorAmps = 0.0;

    /** Current position in meters */
    public double posMeters = 0.0;
    
    /** Current velocity in meters per second */
    public double velMetersPerSec = 0.0;

    /** Current acceleration in meters per second per second */
    public double accMetersPerSecPerSec = 0.0;
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
   * Get current position of linear position controller in meters
   * 
   * @return position of liear position controller in meters
   */
  public double getPos();

  /**
   * Set current position of linear position controller
   * 
   * @param posMeters position in meters
   */
  public void setPos(double posMeters);

  /**
   * Set setpoint of linear position controller
   * 
   * @param posMeters setpoint position in meters
   * @param velMetersPerSec setpoint velocity in meters per second
   */
  public void setSetpoint(double posMeters, double velMetersPerSec);

  /** Called every periodic loop */
  public void periodic();
}
