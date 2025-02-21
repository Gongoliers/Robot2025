package frc.robot.manipulator;

public enum PivotState {
  STOW(0.33203125, 0, true),
  SAFE(0.2, 0, false),
  ALGAE(-0.1, 0, false),
  FLOOR(-0.35, 0, false);

  private final double posRotations;
  private final double velRotationsPerSec;
  private final boolean isUnsafe;
  
  PivotState(
      double posRotations,
      double velRotationsPerSec,
      boolean isUnsafe) {
    
    this.posRotations = posRotations;
    this.velRotationsPerSec = velRotationsPerSec;
    this.isUnsafe = isUnsafe;
  }

  public double getPosRotations() {
    return posRotations;
  }

  public double getVelRotationsPerSec() {
    return velRotationsPerSec;
  }

  public boolean isUnsafe()  {
    return isUnsafe;
  }
}
