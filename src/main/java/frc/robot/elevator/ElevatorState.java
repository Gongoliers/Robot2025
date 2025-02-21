package frc.robot.elevator;

public enum ElevatorState {
  STOW(0, 0),
  L2(0.25, 0),
  L3(0.635, 0),
  L4(1.05, 0),
  MOVING(999, 999);

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
