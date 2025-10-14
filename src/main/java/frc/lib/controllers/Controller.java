package frc.lib.controllers;

/**
 * Represents a generic controller for a mechanism.
 * <p>
 * This controller is responsible for reaching a specified goal by executing
 * control logic during periodic updates. It is parameterized by the type of
 * goal and the type of values that it reports.
 *
 * @param <GoalType> The type of goal that the controller should achieve.
 * @param <ValuesType> The type of values reported by the controller.
 */
public interface Controller<GoalType, ValuesType> {
    /**
     * Configures the hardware used by the controller.
     *
     * @return True if the configuration was successful.
     */
    public boolean configure();

    /**
     * Returns the values from the controller.
     *
     * @return the values from the controller.
     */
    public ValuesType getValues();

    /**
     * Sets the controller goal and performs a control update.
     *
     * @param goal The controller goal.
     */
    public void update(GoalType goal);
}
