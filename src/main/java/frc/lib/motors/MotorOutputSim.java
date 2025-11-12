package frc.lib.motors;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class MotorOutputSim implements MotorOutput {

    private final DCMotorSim sim;

    private final Voltage kS;

    private final Supplier<Voltage> kG;

    /**
     * @param sim The motor system to simulate.
     * @param kS  The voltage loss due to static friction.
     * @param kG  The voltage loss due to gravity.
     */
    public MotorOutputSim(DCMotorSim sim, Voltage kS, Supplier<Voltage> kG) {
        this.sim = sim;
        this.kS = kS;
        this.kG = kG;
    }

    /**
     * @param sim The motor system to simulate.
     * @param kS  The voltage loss due to static friction.
     * @param kG  The voltage loss due to gravity.
     */
    public MotorOutputSim(DCMotorSim sim, Voltage kS, Voltage kG) {
        this(sim, kS, () -> kG);
    }

    /**
     * @param sim The motor system to simulate.
     * @param kS  The voltage loss due to static friction.
     */
    public MotorOutputSim(DCMotorSim sim, Voltage kS) {
        this(sim, kS, Volts::zero);
    }

    /**
     * @param sim The motor system to simulate.
     */
    public MotorOutputSim(DCMotorSim sim) {
        this(sim, Volts.zero());
    }

    @Override
    public void setVoltage(Voltage voltage) {
        double volts = voltage.in(Volts);
        sim.setInputVoltage(volts - calculateVoltageLoss(volts));
    }

    private double calculateVoltageLoss(double voltage) {
        double kS = this.kS.in(Volts);
        double kG = this.kG.get().in(Volts);
        if (Math.abs(voltage) < kS) {
            return kG;
        }
        return Math.copySign(kS, voltage) + kG;
    }

    @Override
    public void updateValues(MotorValues values, Time dt) {
        sim.update(dt.in(Seconds));

        double motorVoltage = sim.getInputVoltage();
        double supplyVoltage = RobotController.getBatteryVoltage();
        double dutyCycle = motorVoltage / supplyVoltage;
        double statorCurrent = sim.getCurrentDrawAmps();
        double supplyCurrent = statorCurrent * dutyCycle;

        values.position.mut_replace(sim.getAngularPositionRad(), Radian);
        values.velocity.mut_replace(
                sim.getAngularVelocityRadPerSec(),
                RadiansPerSecond
        );
        values.acceleration.mut_replace(
                sim.getAngularAccelerationRadPerSecSq(),
                RadiansPerSecondPerSecond
        );
        values.motorVoltage.mut_replace(motorVoltage, Volts);
        values.supplyVoltage.mut_replace(supplyVoltage, Volts);
        values.statorCurrent.mut_replace(statorCurrent, Amps);
        values.supplyCurrent.mut_replace(supplyCurrent, Amps);
    }

    @Override
    public boolean configure() {
        return true;
    }
}
