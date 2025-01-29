package frc.robot.elevator;

public enum ElevatorState {
  STOW(0, 0),
  L1(0.25, 0),
  L2(0.5, 0),
  L3(0.75, 0),
  L4(1, 0);

  private final double posMeters;
  private final double velMetersPerSec;

  ElevatorState(double posMeters, double velMetersPerSec) {
    this.posMeters = posMeters;
    this.velMetersPerSec = velMetersPerSec;
  }

  public double getPosMeters() {
    return posMeters;
  }

  public double getVelMetersPerSec() {
    return velMetersPerSec;
  }
}
