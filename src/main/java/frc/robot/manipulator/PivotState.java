package frc.robot.manipulator;

public enum PivotState {
  STOW(0.33203125, 0),
  SAFE(0.2, 0),
  SCORE(0.1, 0),
  ALGAE(-0.1, 0);

  private final double posRotations;
  private final double velRotationsPerSec;
  
  PivotState(
      double posRotations,
      double velRotationsPerSec) {
    
    this.posRotations = posRotations;
    this.velRotationsPerSec = velRotationsPerSec;
  }

  public double getPosRotations() {
    return posRotations;
  }

  public double getVelRotationsPerSec() {
    return velRotationsPerSec;
  }
}
