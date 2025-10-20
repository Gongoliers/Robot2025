package frc.lib.values;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

/**
 * Represents typical values for a motor.
 * <p>
 * Motor controllers typically output an encoder reading and its derivatives.
 * They also typically output information about the electrical supply.
 */
public class MotorValues {
    public Angle position = Rotations.zero();
    public AngularVelocity velocity = RotationsPerSecond.zero();
    public AngularAcceleration acceleration = RotationsPerSecondPerSecond.zero();
    public Voltage motorVoltage = Volts.zero();
    public Voltage supplyVoltage = Volts.zero();
    public Current statorCurrent = Amps.zero();
    public Current supplyCurrent = Amps.zero();
}
