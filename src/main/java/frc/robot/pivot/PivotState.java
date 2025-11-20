package frc.robot.pivot;

public enum PivotState {
  STOW(0.0),
  OUT(-0.219),
  MOVING(999);

  private final double posRotations;

  PivotState(double posRotations) {
    this.posRotations = posRotations;
  }

  public double getPosRotations() {
    return posRotations;
  }
}
