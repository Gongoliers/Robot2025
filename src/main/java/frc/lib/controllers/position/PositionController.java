package frc.lib.controllers.position;

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

/** Interface that defines the base io functionality all position controllers will inherit */
public interface PositionController {
    
  /** Class that holds logged data from the position controller */
  public static class PositionControllerValues {

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

  /** Configures position controller */
  public void configure();

  /**
   * Gets position controller's updated values and updates values class accordingly
   * 
   * @param values values class to update
   */
  public void getUpdatedVals(PositionControllerValues values);

  /**
  * Gets position of position controller
   * 
   * @return position of positition controller
   */
 public Angle getPos();

  /**
   * Sets position of position controller
   * 
   * @param pos new position
   */
  public void setPos(Angle newPos);

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