package frc.lib.values;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;

/**
 * Represents motor values reported by a TalonFX motor controller.
 */
public class TalonFXValues extends MotorValues {

    private final StatusSignal<Angle> positionSignal;
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<AngularAcceleration> accelerationSignal;
    private final StatusSignal<Voltage> motorVoltageSignal;
    private final StatusSignal<Voltage> supplyVoltageSignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private final StatusSignal<Current> supplyCurrentSignal;

    public TalonFXValues(TalonFX motor) {
        this.positionSignal = motor.getPosition();
        this.velocitySignal = motor.getVelocity();
        this.accelerationSignal = motor.getAcceleration();
        this.motorVoltageSignal = motor.getMotorVoltage();
        this.supplyVoltageSignal = motor.getSupplyVoltage();
        this.statorCurrentSignal = motor.getStatorCurrent();
        this.supplyCurrentSignal = motor.getSupplyCurrent();
    }

    public MotorValues refreshed() {
        BaseStatusSignal.refreshAll(positionSignal, velocitySignal, accelerationSignal, motorVoltageSignal, supplyVoltageSignal, statorCurrentSignal, supplyCurrentSignal);
        this.position = positionSignal.getValue();
        this.velocity = velocitySignal.getValue();
        this.acceleration = accelerationSignal.getValue();
        this.motorVoltage = motorVoltageSignal.getValue();
        this.supplyVoltage = supplyVoltageSignal.getValue();
        this.statorCurrent = statorCurrentSignal.getValue();
        this.supplyCurrent = supplyCurrentSignal.getValue();
        return this;
    }

}
