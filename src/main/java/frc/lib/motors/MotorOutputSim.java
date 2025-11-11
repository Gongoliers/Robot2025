package frc.lib.motors;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class MotorOutputSim extends DCMotorSim implements MotorOutput {

    private final Voltage kS;

    /**
     * @param system  The system to simulate, created by {@link LinearSystemId}.
     * @param gearbox The type of and number of motors in the system.
     * @param kS      The voltage loss due to static friction.
     */
    public MotorOutputSim(LinearSystem<N2, N1, N2> system, DCMotor gearbox, Voltage kS) {
        super(system, gearbox);
        this.kS = kS;
    }

    @Override
    public void setVoltage(Voltage voltage) {
        double effectiveVoltage = calculateEffectiveVoltage(voltage.in(Volts));
        setInputVoltage(effectiveVoltage);
    }

    private double calculateEffectiveVoltage(double voltage) {
        double kS = this.kS.in(Volts);
        if (Math.abs(voltage) < kS) {
            return 0;
        }
        double opposingVoltage = Math.copySign(kS, -voltage);
        return voltage + opposingVoltage;
    }

    @Override
    public void updateValues(MotorValues values, Time dt) {
        update(dt.in(Seconds));

        double motorVoltage = getInputVoltage();
        double supplyVoltage = RobotController.getBatteryVoltage();
        double dutyCycle = motorVoltage / supplyVoltage;
        double statorCurrent = getCurrentDrawAmps();
        double supplyCurrent = statorCurrent * dutyCycle;

        values.position.mut_replace(getAngularPositionRad(), Radian);
        values.velocity.mut_replace(
                getAngularVelocityRadPerSec(),
                RadiansPerSecond
        );
        values.acceleration.mut_replace(
                getAngularAccelerationRadPerSecSq(),
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
