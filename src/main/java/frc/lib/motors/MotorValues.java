package frc.lib.motors;

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

/** Class that contains motor output values */
public class MotorValues {
  
  /** Current position of motor */
  public Angle position = Rotations.of(0.0);

  /** Current velocity of motor */
  public AngularVelocity velocity = RotationsPerSecond.of(0.0);

  /** Current acceleration of motor */
  public AngularAcceleration acceleration = RotationsPerSecondPerSecond.of(0.0);

  /** Current armature voltage  */
  public Voltage motorVoltage = Volts.of(0.0);

  /** Current supply voltage */
  public Voltage supplyVoltage = Volts.of(0.0);

  /** Current stator current */
  public Current statorCurrent = Amps.of(0.0);

  /** Current supply current */
  public Current supplyCurrent = Amps.of(0.0);
  
}
