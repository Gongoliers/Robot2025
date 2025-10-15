package frc.lib;

import edu.wpi.first.units.measure.*;

import static edu.wpi.first.units.Units.*;

/**
 * Represents typical output values for a motor.
 * <p>
 * Motor controllers typically output an encoder reading and its derivatives.
 * They also typically output information about the electrical supply.
 */
public class MotorOutputValues {
    Angle position = Rotations.zero();
    AngularVelocity velocity = RotationsPerSecond.zero();
    AngularAcceleration acceleration = RotationsPerSecondPerSecond.zero();
    Voltage motorVoltage = Volts.zero();
    Voltage supplyVoltage = Volts.zero();
    Current statorCurrent = Amps.zero();
    Current supplyCurrent = Amps.zero();
}