package frc.robot.superstructure;

import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorState;
import frc.robot.intake.IntakeState;
import frc.robot.pivot.Pivot;
import frc.robot.pivot.PivotState;

/** Enum that represents target pivot states and whether or not they are safe */
public enum SuperstructureState {
  STOW(ElevatorState.STOW, PivotState.STOW),
  INTAKE(ElevatorState.STOW, PivotState.STOW),
  L1(ElevatorState.L1, PivotState.STOW),
  L2(ElevatorState.L2, PivotState.OUT),
  L3(ElevatorState.L3, PivotState.OUT),
  L4(ElevatorState.L4, PivotState.OUT),
  ALGAE1(ElevatorState.ALGAE1, PivotState.OUT),
  ALGAE2(ElevatorState.ALGAE2, PivotState.OUT);

  private final ElevatorState elevatorState;
  private final PivotState pivotState;

  private SuperstructureState(ElevatorState elevatorState, PivotState pivotState) {
    this.elevatorState = elevatorState;
    this.pivotState = pivotState;
  }

  /**
   * Gets the elevator state
   * 
   * @return the elevator state
   */
  public ElevatorState getElevatorState() {
    return elevatorState;
  }

  /**
   * Gets the pivot state
   * 
   * @return the pivot state
   */
  public PivotState getPivotState() {
    return pivotState;
  }
}