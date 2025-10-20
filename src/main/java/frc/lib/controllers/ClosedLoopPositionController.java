package frc.lib.controllers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.outputs.Output;
import frc.lib.values.MotorValues;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class ClosedLoopPositionController implements Controller<Angle, MotorValues> {

    private final Output<Voltage, MotorValues> output;

    private final PIDController pid;

    private final Supplier<Voltage> feedforward;

    public ClosedLoopPositionController(Output<Voltage, MotorValues> output, Per<VoltageUnit, AngleUnit> kP, Per<VoltageUnit, AngularVelocityUnit> kD, Supplier<Voltage> feedforward) {
        this(output, new PIDController(kP.in(Volts.per(Rotation)), 0, kD.in(Volts.per(RotationsPerSecond))), feedforward);
    }

    public ClosedLoopPositionController(Output<Voltage, MotorValues> output, PIDController pid, Supplier<Voltage> feedforward) {
        this.output = output;
        this.pid = pid;
        this.feedforward = feedforward;
    }

    @Override
    public MotorValues getOutputValues() {
        return this.output.getOutputValues();
    }

    @Override
    public void update(Angle goal) {
        var measurement = getOutputValues().position.in(Rotations);
        var setpoint = goal.in(Rotations);
        Voltage feedbackVoltage = Volts.of(pid.calculate(measurement, setpoint));
        Voltage feedforwardVoltage = feedforward.get();
        this.output.update(feedbackVoltage.plus(feedforwardVoltage));
    }
}
