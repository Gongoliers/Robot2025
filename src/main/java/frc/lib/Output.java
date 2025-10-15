package frc.lib;

/**
 * A generic interface for applying control inputs.
 * <p>
 * An {@code Output} is responsible for interfacing with hardware and providing
 * information about the state of the system. It accepts control inputs from
 * a {@link frc.lib.controllers.Controller} and returns values that represent
 * the system state.
 *
 * @param <ControlInputType> The type of control input that the output receives.
 * @param <OutputValuesType> The type of output values returned by the output.
 */
public interface Output<ControlInputType, OutputValuesType> {
    /**
     * Configures the output for operation.
     * <p>
     * This method initializes hardware and performs other required setup before
     * the system can accept control inputs.
     *
     * @return {@code true} if configuration succeeded, {@code false} otherwise.
     */
    boolean configure();

    /**
     * Returns the output values of the system.
     *
     * @return the output values of the system.
     */
    OutputValuesType getOutputValues();

    /**
     * Updates the output with a control input.
     * <p>
     * This method should be called once per periodic iteration to maintain
     * continuous control.
     *
     * @param input The input to apply to the output.
     */
    void update(ControlInputType input);
}
