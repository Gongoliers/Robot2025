package frc.lib.controllers.velocity;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

/** Interface that defines the base io functionality all velocity controllers will inherit */
public interface VelocityController {

  /** Class that holds logged data from the position controller */
  public static class VelocityControllerValues {

    /** Motor position */
    public MutAngle position = Rotations.mutable(0.0);

    /** Motor velocity */
    public MutAngularVelocity velocity = RotationsPerSecond.mutable(0.0);

    /** Motor acceleration */
    public MutAngularAcceleration acceleration = RotationsPerSecondPerSecond.mutable(0.0);

    /** Motor voltage in volts */
    public MutVoltage motorVoltage = Volts.mutable(0.0);

    /** Stator current in amps */
    public MutCurrent statorCurrent = Amps.mutable(0.0);

    /** Supply current in amps */
    public MutCurrent supplyCurrent = Amps.mutable(0.0);
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

  /**
   * Sets a manual voltage for the position controller
   *
   * @param volts manual voltage to set
   */
  public void setVoltage(Voltage volts);

  /** Clears a set voltage */
  public void clearVoltage();

  /** Called every periodic loop */
  public void periodic();
}
