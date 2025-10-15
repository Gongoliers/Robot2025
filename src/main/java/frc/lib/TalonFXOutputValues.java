package frc.lib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;

/**
 * Represents motor values reported by  a TalonFX motor controller.
 */
public class TalonFXOutputValues {

    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
    private final StatusSignal<AngularAcceleration> acceleration;
    private final StatusSignal<Voltage> motorVoltage;
    private final StatusSignal<Voltage> supplyVoltage;
    private final StatusSignal<Current> statorCurrent;
    private final StatusSignal<Current> supplyCurrent;

    public TalonFXOutputValues(TalonFX motor) {
        this.position = motor.getPosition();
        this.velocity = motor.getVelocity();
        this.acceleration = motor.getAcceleration();
        this.motorVoltage = motor.getMotorVoltage();
        this.supplyVoltage = motor.getSupplyVoltage();
        this.statorCurrent = motor.getStatorCurrent();
        this.supplyCurrent = motor.getSupplyCurrent();
    }

    public TalonFXOutputValues refresh() {
        BaseStatusSignal.refreshAll(position, velocity, acceleration, motorVoltage, supplyVoltage, statorCurrent, supplyCurrent);
        return this;
    }

    public MotorOutputValues toMotorOutputValues() {
        MotorOutputValues output = new MotorOutputValues();
        output.position = position.getValue();
        output.velocity = velocity.getValue();
        output.acceleration = acceleration.getValue();
        output.motorVoltage = motorVoltage.getValue();
        output.supplyVoltage = supplyVoltage.getValue();
        output.statorCurrent = statorCurrent.getValue();
        output.supplyCurrent = supplyCurrent.getValue();
        return output;
    }

}
