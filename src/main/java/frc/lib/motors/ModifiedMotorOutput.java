package frc.lib.motors;

import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

import java.util.function.Function;
import static edu.wpi.first.units.Units.Volts;

public class ModifiedMotorOutput implements MotorOutput {

    private final MotorOutput motorOutput;

    private final Function<Voltage, Voltage> modifier;

    private final MutVoltage voltage;

    public ModifiedMotorOutput(MotorOutput motorOutput, Function<Voltage, Voltage> modifier) {
        this.motorOutput = motorOutput;
        this.modifier = modifier;
        this.voltage = Volts.mutable(0);
    }

    @Override
    public void setVoltage(Voltage voltage) {
        this.voltage.mut_replace(voltage);
        Voltage modifiedVoltage = modifier.apply(voltage);
        motorOutput.setVoltage(modifiedVoltage);
    }

    @Override
    public void updateValues(MotorValues values, Time dt) {
        motorOutput.updateValues(values, dt);
        values.motorVoltage.mut_replace(voltage);
    }

    @Override
    public boolean configure() {
        return motorOutput.configure();
    }
}
