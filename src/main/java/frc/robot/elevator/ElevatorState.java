package frc.robot.elevator;

public enum ElevatorState {
  STOW(0),
  L1(1);

  private final double elevatorPosMeters;

  ElevatorState(double elevatorPosMeters) {
    this.elevatorPosMeters = elevatorPosMeters;
  }

  public double getElevatorPosMeters() {
    return elevatorPosMeters;
  }
}
