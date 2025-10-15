package frc.lib.controllers;

/**
 * A generic interface for implementing control logic for a system.
 * <p>
 * A {@code Controller} is responsible for driving a system to a goal by performing
 * periodic control updates. It accepts a goal that the system should achieve, and
 * returns output values that represent the system state.
 *
 * @param <ControllerGoalType> The type of the goal that the controller should achieve.
 * @param <OutputValuesType> The type of the output values returned by the controller.
 */
public interface Controller<ControllerGoalType, OutputValuesType> {
    /**
     * Returns the output values of the system.
     *
     * @return the output values of the system.
     */
    OutputValuesType getOutputValues();

    /**
     * Performs a control update to achieve the goal.
     * <p>
     * This method should be called once per periodic iteration to maintain
     * continuous control, regardless of whether the goal has changed.
     *
     * @param goal The goal for the controller to achieve.
     */
    void update(ControllerGoalType goal);
}
