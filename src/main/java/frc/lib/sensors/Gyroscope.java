package frc.lib.sensors;

/**
 * Basic interface for gyroscopes
 * 
 * Outlines required functions and variables all gyroscopes have to make implementing new types
 * of gyroscopes in the future much simpler
 */
public interface Gyroscope {
  
  /** Gyroscope values */
  public static class GyroscopeValues {

    /** Roll angle in rotations */
    public double rollRotations = 0.0;

    /** Pitch angle in rotations */
    public double pitchRotations = 0.0;

    /** Yaw angle in rotations */
    public double yawRotations = 0.0;

    /** Roll angle velocity in rotations per second */
    public double rollVelRotationsPerSec = 0.0;

    /** Pitch angle velocity in rotations per second */
    public double pitchVelRotationsPerSec = 0.0;

    /** Yaw angle in rotations per second */
    public double yawVelRotationsPerSec = 0.0;
  }

  /** Configures the gyroscope */
  public void configure();

  /**
   * Get the gyroscope's updated values and update the provided values class accordingly
   * 
   * @param values values class to be updated
   */
  public void getUpdatedVals(GyroscopeValues values);

  /**
   * Sets the gyroscope's yaw in rotations
   * 
   * @param yawRotations yaw in rotations
   */
  public void setYaw(double yawRotations);

  /** Called every periodic loop */
  public void periodic();
}
