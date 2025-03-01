package frc.robot.pivot;

/** Enum that represents target pivot states and whether or not they are safe */
public enum PivotState {
  STOW(0.0, false),
  TEST(0.5, false),
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
