package frc.lib.motors;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/**
 * Simple interface that abstracts motor hardware (allowing us to sent a control request to a motor
 * and get values back, no matter if it's one motor, two motors, or a simulated motor)
 */
public interface MotorOutput {

  // TODO: In the future, maybe find a way to allow for other kinds of phoenix specific control
  // requests for things like PositionVoltage for swerve steer motors
  /**
   * Sets the voltage of the motor output
   *
   * @param voltage new set voltage
   */
  public void setVoltage(Voltage voltage);

  /**
   * Sets the position of the motor output by setting an offset always added to output position
   *
   * @param newPostiion new position
   */
  public void setPosition(Angle newPostiion);

  /**
   * Updates values stored in a MotorValues class and updates sim (if there is a sim)
   *
   * @param values values class to update
   * @param dt delta time to allow for sim calculation of values or values that require dt to be
   *     calculated
   */
  public void updateValues(MotorValues values, Time dt);

  /**
   * Configures motor hardware
   *
   * @return true if configuration was successful
   */
  public boolean configure();
}
