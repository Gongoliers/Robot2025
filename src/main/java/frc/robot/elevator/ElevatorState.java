package frc.robot.elevator;

public enum ElevatorState {
  STOW(0),
  L1(0.15),
  HARDSTOP(0.15),
  L2(0.25),
  L3(1.05),
  L4(1.57),
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