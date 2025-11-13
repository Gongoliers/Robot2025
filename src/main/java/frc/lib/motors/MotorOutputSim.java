package frc.lib.motors;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import java.util.function.Function;
import java.util.function.Supplier;

public class MotorOutputSim implements MotorOutput {

    private final DCMotorSim sim;

    private final MutVoltage voltage;

    private final Function<Voltage, Voltage> modifier;

    /**
     * Creates a simulated motor system with a voltage modifier.
     *
     * @param sim      The motor system to simulate.
     * @param modifier The function that modifies the voltage.
     */
    public MotorOutputSim(DCMotorSim sim, Function<Voltage, Voltage> modifier) {
        this.sim = sim;
        this.voltage = Volts.mutable(0);
        this.modifier = modifier;
    }

    /**
     * Creates a simulated motor system without a voltage modifier.
     *
     * @param sim The motor system to simulate.
     */
    public MotorOutputSim(DCMotorSim sim) {
        this(sim, v -> v);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        this.voltage.mut_replace(voltage);
        double effectiveVoltage = modifier.apply(voltage).in(Volts);
        sim.setInputVoltage(effectiveVoltage);
    }

    @Override
    public void updateValues(MotorValues values, Time dt) {
        sim.update(dt.in(Seconds));

        double motorVoltage = voltage.in(Volts);
        double supplyVoltage = RobotController.getBatteryVoltage();
        double dutyCycle = motorVoltage / supplyVoltage;
        double statorCurrent = sim.getCurrentDrawAmps();
        double supplyCurrent = statorCurrent * dutyCycle;

        values.position.mut_replace(sim.getAngularPositionRad(), Radian);
        values.velocity.mut_replace(sim.getAngularVelocityRadPerSec(), RadiansPerSecond);
        values.acceleration.mut_replace(
                sim.getAngularAccelerationRadPerSecSq(), RadiansPerSecondPerSecond);
        values.motorVoltage.mut_replace(voltage);
        values.supplyVoltage.mut_replace(supplyVoltage, Volts);
        values.statorCurrent.mut_replace(statorCurrent, Amps);
        values.supplyCurrent.mut_replace(supplyCurrent, Amps);
    }

    @Override
    public boolean configure() {
        return true;
    }
}
