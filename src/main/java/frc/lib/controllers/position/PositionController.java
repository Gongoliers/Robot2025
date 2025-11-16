package frc.lib.controllers.position;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;

/** Interface that defines the base io functionality all position controllers will inherit */
public interface PositionController {

  /** Class that holds logged data from the position controller */
  public static class PositionControllerValues {

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

  /** Configures position controller */
  public void configure();

  /**
   * Gets position controller's updated values and updates values class accordingly
   *
   * @param values values class to update
   */
  public void getUpdatedVals(PositionControllerValues values);

  /**
   * Sets position of position controller
   *
   * @param pos new position
   */
  public void setPosition(Angle newPos);

  /**
   * Sets setpoint of position controller
   *
   * @param pos setpoint position
   * @param vel setpoint velocity
   */
  public void setSetpoint(Angle pos, AngularVelocity vel);

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
