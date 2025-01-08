package frc.lib.controllers.velocity;

/**
 * Basic interface for velocity controllers
 * 
 * Outlines required functions and variables all velocity controllers have to make implementing new types
 * of velocity controllers in the future much simpler
 */
public interface VelocityController {
  
  /** Velocity controller values */
  public static class VelocityControllerValues {

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

  /** Configures the velocity controller */
  public void configure();

  /**
   * Get the velocity controller's updated values and update the provided values class accordingly
   * 
   * @param values values class to be updated
   */
  public void getUpdatedVals(VelocityControllerValues values);

  /**
   * Sets the position of the controller
   * 
   * @param posRotations position in rotations
   */
  public void setPos(double posRotations);

  /**
   * Sets the setpoint (target velocity) of the velocity controller
   * 
   * @param velRotationsPerSec target velocity in rotations per second
   */
  public void setSetpoint(double velRotationsPerSec);

  /** Called every periodic loop */
  public void periodic();
}
