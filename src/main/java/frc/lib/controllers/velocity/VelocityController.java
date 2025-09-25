package frc.lib.controllers.velocity;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

/** Interface that defines the base io functionality all velocity controllers will inherit */
public interface VelocityController {
  
  /** Class that holds logged data from the position controller */
  public static class VelocityControllerValues {

    /** Motor position */
    public Angle position = Rotations.of(0.0);

    /** Motor velocity */
    public AngularVelocity velocity = RotationsPerSecond.of(0.0);

    /** Motor acceleration */
    public AngularAcceleration acceleration = RotationsPerSecondPerSecond.of(0.0);

    /** Motor voltage in volts */
    public Voltage motorVoltage = Volts.of(0.0);

    /** Stator current in amps */
    public Current statorCurrent = Amps.of(0.0);

    /** Supply current in amps */
    public Current supplyCurrent = Amps.of(0.0);

  }

  /** Configures the velocity controller */
  public void configure();

  /**
   * Gets velocity controller's updated values and updates values class accordingly
   * 
   * @param values values class to update
   */
  public void getUpdatedVals(VelocityControllerValues values);

  /**
   * Sets setpoint of velocity controller
   * 
   * @param velocity target velocity
   */
  public void setSetpoint(AngularVelocity velocity);

  /** Called every periodic loop */
  public void periodic();
}
