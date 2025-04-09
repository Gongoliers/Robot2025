package frc.robot.endgame;

/** Enum that represents target endgame states and whether or not they are safe */
public enum EndgameState {
  ARMED(0.35, false),
  HOOK(0.15, false),
  STOW(-0.25, true),
  MOVING(999, false);

  private final double posRotations;
  private final boolean safe;

  private EndgameState(double posRotations, boolean isSafe) {
    this.posRotations = posRotations;
    this.safe = isSafe;
  }

  /**
   * Gets the position in rotations of the endgame state
   * 
   * @return the position in rotations of the endgame state
   */
  public double getPosRotations() {
    return posRotations;
  }

  /**
   * Returns true if the endgame state is safe
   * 
   * @return true if the endgame state is safe
   */
  public boolean isSafe() {
    return safe;
  }

  /**
   * Returns true if the endgame state is unsafe
   * 
   * @return true if the endgame state is unsafe
   */
  public boolean isUnsafe() {
    return !safe;
  }
}
