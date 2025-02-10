package frc.robot.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.robot.elevator.Elevator;
import frc.robot.manipulator.Manipulator;
import frc.robot.manipulator.PivotState;

/** Superstructure subsystem */
public class Superstructure extends Subsystem {
  
  /** Superstructure subsystem singleton */
  private static Superstructure instance = null;

  /** Elevator reference */
  private final Elevator elevator;

  /** Manipulator reference */
  private final Manipulator manipulator;

  /** Superstructure state */
  private SuperstructureState currentState;

  /** Initializes superstructure subsystem */
  private Superstructure() {
    elevator = Elevator.getInstance();
    manipulator = Manipulator.getInstance();

    currentState = SuperstructureState.STOW;
  }

  /**
   * Gets superstructure subsystem instance if there is one, and creates and returns one if there isn't
   * 
   * @return superstructure subsystem instance
   */
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }

    return instance;
  }

  @Override
  public void periodic() {

  }

  @Override
  public void initializeTab() {

  }

  public Command safelyTo(SuperstructureState targetState) {
    return Commands
      .runOnce(
        () -> manipulator.setTargetPivotState(PivotState.SAFE))
          .until(manipulator::atTargetPivotState)
      .andThen(
        () -> elevator.setTargetState(targetState.getElevatorState()))
          .until(elevator::atTargetState)
      .andThen(
        () -> {
          manipulator.setTargetPivotState(targetState.getPivotState());
          manipulator.setTargetIntakeState(targetState.getIntakeState());
        });
  }
}
