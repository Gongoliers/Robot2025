package frc.lib.controllers;

import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.MotorOutputValues;
import frc.lib.TalonFXOutputValues;

/**
 * Represents a controller for a mechanism that reaches a velocity using voltage
 * input to a TalonFX motor controller.
 * <p>
 * This class represents one of the simplest open-loop or feedforward controllers
 * for a DC motor. The steady-state speed ω of a DC motor for a positive voltage V is
 * given by ω = kV × (V - kS), where kV is the amount of angular velocity per volt, and 
 * kS is the amount of voltage to overcome static friction. Using this equation to solve
 * for V, the amount of voltage required to reach a steady-state speed ω is kS + V ÷ kV.
 * Often, V ÷ kV is written as the multiplication kV × V, where kV is inverted.
 */
public class OpenLoopVelocityController implements Controller<AngularVelocity, MotorOutputValues> {

    private final TalonFX motor;

    private final TalonFXOutputValues outputValues;

    private final VoltageOut control;

    private final Voltage kS;

    private final Per<VoltageUnit, AngularVelocityUnit> kV;

    public OpenLoopVelocityController(
            TalonFX motor, Voltage kS, Per<VoltageUnit, AngularVelocityUnit> kV) {
        this.motor = motor;
        this.outputValues = new TalonFXOutputValues(this.motor);
        this.kS = kS;
        this.kV = kV;
        this.control = new VoltageOut(0.0);
    }

    @Override
    public boolean configure() {
        return true;
    }

    @Override
    public MotorOutputValues getOutputValues() {
        return this.outputValues.refresh().toMotorOutputValues();
    }

    @Override
    public void update(AngularVelocity goal) {
        var direction = Math.signum(goal.baseUnitMagnitude());
        Voltage voltage = kS.times(direction).plus(kV.timesDivisor(goal));
        motor.setControl(control.withOutput(voltage));
    }
}
