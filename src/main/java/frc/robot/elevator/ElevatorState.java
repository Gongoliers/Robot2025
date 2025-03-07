package frc.robot.elevator;

public enum ElevatorState {
  STOW(0),
  INTAKE(0.03),
  L1(0.1),
  L2(0.25),
  L3(0.635),
  L4(1.05),
  ALGAE1(0.175),
  ALGAE2(0.4),
  MOVING(999);

  private final double posMeters;

  ElevatorState(double posMeters) {
    this.posMeters = posMeters;
  }

  public double getPosMeters() {
    return posMeters;
  }
}
