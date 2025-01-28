package frc.robot.elevator;

public enum ElevatorState {
  STOW(0, 0),
  L1(0.5, 0);

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
