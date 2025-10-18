package frc.lib.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.outputs.Output;
import frc.lib.values.MotorValues;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

public class ClosedLoopPositionController implements Controller<Angle, MotorValues> {

    private final Output<Voltage, MotorValues> output;

    private final PIDController pid;

    public ClosedLoopPositionController(Output<Voltage, MotorValues> output, PIDController pid) {
        this.output = output;
        this.pid = pid;
    }

    @Override
    public MotorValues getOutputValues() {
        return this.output.getOutputValues();
    }

    @Override
    public void update(Angle goal) {
        var measurement = getOutputValues().position.in(Rotations);
        var setpoint = goal.in(Rotations);
        Voltage voltage = Volts.of(pid.calculate(measurement, setpoint));
        this.output.update(voltage);
    }
}
