package frc.robot.superstructure;

import frc.robot.elevator.ElevatorState;
import frc.robot.manipulator.IntakeState;
import frc.robot.manipulator.PivotState;

public enum SuperstructureState {
  STOW(ElevatorState.STOW, PivotState.STOW, IntakeState.STOP);

  private final ElevatorState elevatorState;
  private final PivotState pivotState;
  private final IntakeState intakeState;

  SuperstructureState(ElevatorState elevatorState, PivotState pivotState, IntakeState intakeState) {
    this.elevatorState = elevatorState;
    this.pivotState = pivotState;
    this.intakeState = intakeState;
  }

  public ElevatorState getElevatorState() {
    return elevatorState;
  }

  public PivotState getPivotState() {
    return pivotState;
  }

  public IntakeState getIntakeState() {
    return intakeState;
  }
}
