package frc.lib;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Voltage;

/**
 * A TalonFX that accepts voltage as input and returns motor values.
 */
public class TalonFXVoltageOutput implements Output<Voltage, MotorValues> {

    /**
     * The TalonFX that accepts the voltage.
     */
    private final TalonFX motor;

    /**
     * The output values returned by the TalonFX.
     */
    private final TalonFXValues outputValues;

    /**
     * The invert state of the TalonFX, as seen from the front of the motor.
     */
    private final InvertedValue inverted;

    /**
     * The control request used by the TalonFX.
     */
    private final VoltageOut control;

    /**
     * Creates an output with a TalonFX that accepts voltages.
     *
     * @param motor  The TalonFX that accepts voltages.
     * @param invert {@code true} if the TalonFX is inverted, {@code false} otherwise.
     */
    public TalonFXVoltageOutput(TalonFX motor, boolean invert) {
        this.motor = motor;
        this.outputValues = new TalonFXValues(this.motor);
        this.inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
        this.control = new VoltageOut(0.0);
    }

    @Override
    public boolean configure() {
        TalonFXConfigurator configurator = this.motor.getConfigurator();
        MotorOutputConfigs config = new MotorOutputConfigs().withInverted(this.inverted);
        StatusCode statusCode = configurator.apply(config);
        return statusCode.isOK();
    }

    @Override
    public MotorValues getOutputValues() {
        return outputValues.refreshed();
    }

    @Override
    public void update(Voltage input) {
        motor.setControl(this.control.withOutput(input));
    }
}
