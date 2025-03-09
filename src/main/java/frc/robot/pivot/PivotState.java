package frc.robot.pivot;

/** Enum that represents target pivot states and whether or not they are safe */
public enum PivotState {
  STOW(0.34, false),
  SCORE(0.23, false),
  SCOREL4(0.16, false),
  INTAKE(0.34, false),
  FLOORALGAE(0.12, true),
  L1(0.3, true),
  ALGAE(0.2, false),
  SAFE(0.29, true),
  MOVING(999, false);

  private final double posRotations;
  private final boolean safe;

  private PivotState(double posRotations, boolean isSafe) {
    this.posRotations = posRotations;
    this.safe = isSafe;
  }

  /**
   * Gets the position in rotations of the pivot state
   * 
   * @return the position in rotations of the pivot state
   */
  public double getPosRotations() {
    return posRotations;
  }

  /**
   * Returns true if the pivot state is safe
   * 
   * @return true if the pivot state is safe
   */
  public boolean isSafe() {
    return safe;
  }

  /**
   * Returns true if the pivot state is unsafe
   * 
   * @return true if the pivot state is unsafe
   */
  public boolean isUnsafe() {
    return !safe;
  }
}
