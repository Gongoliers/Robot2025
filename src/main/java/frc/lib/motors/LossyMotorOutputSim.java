package frc.lib.motors;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.Function;

/** This class is a decorator around a simulated motor output that adds static friction and gravity. */
public class LossyMotorOutputSim implements MotorOutput {

    private final MotorOutput sim;

    private final Voltage kS;

    private final Function<Angle, Voltage> kG;

    private final MutVoltage motorVoltage;

    private final MutVoltage effectiveMotorVoltage;

    private final MutAngle position;

    /**
     * Creates a simulated motor system with a constant static friction voltage loss and a possibly
     * variable gravity voltage loss.
     *
     * @param sim The motor system to simulate.
     * @param kS The voltage loss due to static friction.
     * @param kG The voltage loss due to gravity, dependent on motor position.
     */
    public LossyMotorOutputSim(
        MotorOutputSim sim,
        Voltage kS,
        Function<Angle, Voltage> kG
    ) {
        this.sim = sim;
        this.kS = kS;
        this.kG = kG;
        this.motorVoltage = Volts.mutable(0);
        this.effectiveMotorVoltage = Volts.mutable(0);
        this.position = Radians.mutable(0);
    }

    /**
     * Creates a simulated motor system with a constant static friction voltage loss and a constant
     * gravity voltage loss.
     *
     * @param sim The motor system to simulate.
     * @param kS The voltage loss due to static friction.
     * @param kG The voltage loss due to gravity, independent of motor position.
     */
    public LossyMotorOutputSim(MotorOutputSim sim, Voltage kS, Voltage kG) {
        this(sim, kS, position -> kG);
    }

    /**
     * Creates a simulated motor system with a constant static friction voltage loss and no gravity
     * voltage loss.
     *
     * @param sim The motor system to simulate.
     * @param kS The voltage loss due to static friction.
     */
    public LossyMotorOutputSim(MotorOutputSim sim, Voltage kS) {
        this(sim, kS, Volts.zero());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        this.motorVoltage.mut_replace(voltage);
        double volts = voltage.in(Volts);
        effectiveMotorVoltage.mut_replace(
            volts - calculateVoltageLoss(volts),
            Volts
        );
        // Override the requested voltage with the effective voltage
        sim.setVoltage(effectiveMotorVoltage);
    }

    /**
     * Calculates the voltage losses due to static friction and gravity.
     *
     * @param voltage The voltage being applied to the system, prior to any losses.
     * @return The voltage loss due to static friction and gravity.
     */
    private double calculateVoltageLoss(double voltage) {
        double kS = this.kS.in(Volts);
        double kG = this.kG.apply(position).in(Volts);
        if (Math.abs(voltage) < kS) {
            return kG;
        }
        return Math.copySign(kS, voltage) + kG;
    }

    @Override
    public void updateValues(MotorValues values, Time dt) {
        sim.updateValues(values, dt);
        // Update the position so the calculated kG will be accurate
        this.position.mut_replace(values.position);
        // Override the motor voltage with the actual requested voltage
        values.motorVoltage.mut_replace(motorVoltage);
    }

    @Override
    public boolean configure() {
        return sim.configure();
    }
}
