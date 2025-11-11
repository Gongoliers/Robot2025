package frc.lib.motors;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import static edu.wpi.first.units.Units.*;

public class MotorOutputSim extends DCMotorSim implements MotorOutput {

    private final double kS;

    public MotorOutputSim(double kV, double kA, DCMotor motor, double kS) {
        super(LinearSystemId.identifyPositionSystem(kV, kA), motor);
        this.kS = kS;
    }

    @Override
    public void setVoltage(Voltage voltage) {
        double effectiveVoltage = calculateEffectiveVoltage(voltage.in(Volts));
        setInputVoltage(effectiveVoltage);
    }

    private double calculateEffectiveVoltage(double voltage) {
        if (Math.abs(voltage) < this.kS) {
            return 0;
        }
        double opposingVoltage = Math.copySign(this.kS, -voltage);
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
