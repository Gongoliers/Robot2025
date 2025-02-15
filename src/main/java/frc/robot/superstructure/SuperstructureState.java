package frc.robot.superstructure;

import frc.robot.elevator.ElevatorState;
import frc.robot.manipulator.IntakeState;
import frc.robot.manipulator.PivotState;

public enum SuperstructureState {
  STOW(ElevatorState.STOW, PivotState.STOW, IntakeState.STOP),
  L1(ElevatorState.L1, PivotState.SAFE, IntakeState.STOP),
  L2(ElevatorState.L2, PivotState.SAFE, IntakeState.STOP),
  L3(ElevatorState.L3, PivotState.SAFE, IntakeState.STOP),
  L4(ElevatorState.L4, PivotState.SAFE, IntakeState.STOP);

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
