package frc.lib.controllers;

/**
 * Represents a generic controller for a mechanism.
 * <p>
 * This controller is responsible for reaching a specified goal by executing
 * control logic during periodic updates. It is parameterized by the type of
 * goal and the type of values that it outputs.
 *
 * @param <ControllerGoalType> The type of goal that the controller should achieve.
 * @param <OutputValuesType> The type of values reported by the controller.
 */
public interface Controller<ControllerGoalType, OutputValuesType> {
    /**
     * Configures the hardware used by the controller.
     *
     * @return True if the configuration was successful.
     */
    boolean configure();

    /**
     * Returns the output values from the controller.
     *
     * @return the output values from the controller.
     */
    OutputValuesType getOutputValues();

    /**
     * Sets the controller goal and performs a control update.
     *
     * @param goal The controller goal.
     */
    void update(ControllerGoalType goal);
}
