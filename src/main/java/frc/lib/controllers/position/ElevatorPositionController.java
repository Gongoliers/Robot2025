package frc.lib.controllers.position;

/** 
 * Basic interface for elevator position controllers
 * 
 * Outlines required functions and variables all elevator position controllers have to make implementing new types
 * of position controllers in the future much simpler
 */
public interface ElevatorPositionController {
  
  /** elevator position controller values */
  public static class ElevatorPositionControllerValues {

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

  /** Configures the elevator position controller */
  public void configure();

  /**
   * Get the elevator position controller's updated values and update the provided values class accordingly
   * 
   * @param values values class to be updated
   */
  public void getUpdatedVals(ElevatorPositionControllerValues values);

  /**
   * Get current position of elevator position controller in meters
   * 
   * @return position of liear position controller in meters
   */
  public double getElevatorPos();

  /**
   * Set current position of elevator position controller
   * 
   * @param posMeters position in meters
   */
  public void setElevatorPos(double posMeters);

  /**
   * Set setpoint of elevator position controller
   * 
   * @param posMeters setpoint position in meters
   * @param velMetersPerSec setpoint velocity in meters per second
   */
  public void setSetpoint(double posMeters, double velMetersPerSec);

  /** Called every periodic loop */
  public void periodic();
}
