package frc.lib.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;

/** Class that contains motor output values */
public class MotorValues {

  /** Current position of motor */
  public MutAngle position = Rotations.mutable(0.0);

  /** Current velocity of motor */
  public MutAngularVelocity velocity = RotationsPerSecond.mutable(0.0);

  /** Current acceleration of motor */
  public MutAngularAcceleration acceleration = RotationsPerSecondPerSecond.mutable(0.0);

  /** Current armature voltage */
  public MutVoltage motorVoltage = Volts.mutable(0.0);

  /** Current supply voltage */
  public MutVoltage supplyVoltage = Volts.mutable(0.0);

  /** Current stator current */
  public MutCurrent statorCurrent = Amps.mutable(0.0);

  /** Current supply current */
  public MutCurrent supplyCurrent = Amps.mutable(0.0);
}
