package frc.lib.controllers;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.MotorValues;
import frc.lib.Output;

/**
 * Represents a controller for a mechanism that reaches a velocity using open loop
 * voltage control.
 * <p>
 * This class represents one of the simplest open-loop or feedforward controllers
 * for a DC motor. The steady-state speed ω of a DC motor for a positive voltage V is
 * given by ω = kV × (V - kS), where kV is the amount of angular velocity per volt, and 
 * kS is the amount of voltage to overcome static friction. Using this equation to solve
 * for V, the amount of voltage required to reach a steady-state speed ω is kS + V ÷ kV.
 * Often, V ÷ kV is written as the multiplication kV × V, where kV is inverted.
 */
public class OpenLoopVelocityController implements Controller<AngularVelocity, MotorValues> {

    private final Output<Voltage, MotorValues> output;

    private final Voltage kS;

    private final Per<VoltageUnit, AngularVelocityUnit> kV;

    public OpenLoopVelocityController(
            Output<Voltage, MotorValues> output, Voltage kS, Per<VoltageUnit, AngularVelocityUnit> kV) {
        this.output = output;
        this.kS = kS;
        this.kV = kV;
    }

    @Override
    public MotorValues getOutputValues() {
        return this.output.getOutputValues();
    }

    @Override
    public void update(AngularVelocity goal) {
        var direction = Math.signum(goal.baseUnitMagnitude());
        Voltage voltage = kS.times(direction).plus(kV.timesDivisor(goal));
        output.update(voltage);
    }
}
