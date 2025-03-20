package frc.robot.elevator;

public enum ElevatorState {
  STOW(0),
  INTAKE(0.15),
  L1(0.1),
  L2(0.35),
  L3(0.68),
  L4(1.41),
  ALGAE1(0.45),
  ALGAE2(0.85),
  MOVING(999);

  private final double posMeters;

  ElevatorState(double posMeters) {
    this.posMeters = posMeters;
  }

  public double getPosMeters() {
    return posMeters;
  }
}
