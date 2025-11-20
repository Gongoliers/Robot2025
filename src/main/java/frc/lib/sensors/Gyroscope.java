package frc.lib.sensors;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;

/**
 * Basic interface for gyroscopes
 *
 * <p>Outlines required functions and variables all gyroscopes have to make implementing new types
 * of gyroscopes in the future much simpler
 */
public interface Gyroscope {

  /** Gyroscope values */
  public static class GyroscopeValues {

    /** Roll angle */
    public Angle roll = Rotations.of(0.0);

    /** Pitch angle */
    public Angle pitch = Rotations.of(0.0);

    /** Yaw angle */
    public Angle yaw = Rotations.of(0.0);

    /** Roll angle velocity */
    public AngularVelocity rollVelocity = RotationsPerSecond.of(0.0);

    /** Pitch angle velocity */
    public AngularVelocity pitchVelocity = RotationsPerSecond.of(0.0);

    /** Yaw angle velocity */
    public AngularVelocity yawVelociy = RotationsPerSecond.of(0.0);
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
   * Sets the gyroscope's yaw
   *
   * @param newYaw new gyroscope yaw
   */
  public void setYaw(Angle newYaw);

  /** Called every periodic loop */
  public void periodic();
}
