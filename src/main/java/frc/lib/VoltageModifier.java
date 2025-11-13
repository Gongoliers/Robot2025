package frc.lib;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

import java.util.function.Function;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

public class VoltageModifier {

    public static Function<Voltage, Voltage> staticFriction(Voltage kS) {
        return voltage -> {
            if (voltage.abs(Volts) < kS.in(Volts)) {
                return Volts.zero();
            }

            double direction = Math.signum(voltage.in(Volts));
            return voltage.minus(kS.times(direction));
        };
    }

    public static Function<Voltage, Voltage> elevatorGravity(Voltage kG) {
        return voltage -> kG.times(-1);
    }

    public static Function<Voltage, Voltage> armGravity(Voltage kG, Supplier<Angle> angle) {
        return voltage -> {
            double cos = Math.cos(angle.get().in(Radians));
            return kG.times(-cos);
        };
    }

    public static Function<Voltage, Voltage> forElevator(Voltage kS, Voltage kG) {
        Function<Voltage, Voltage> staticFriction = staticFriction(kS);
        Function<Voltage, Voltage> gravity = elevatorGravity(kG);

        return voltage -> staticFriction.apply(voltage).plus(gravity.apply(voltage));
    }

    public static Function<Voltage, Voltage> forArm(Voltage kS, Voltage kG, Supplier<Angle> angle) {
        Function<Voltage, Voltage> staticFriction = staticFriction(kS);
        Function<Voltage, Voltage> gravity = armGravity(kG, angle);

        return voltage -> staticFriction.apply(voltage).plus(gravity.apply(voltage));
    }

}
