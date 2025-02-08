package frc.robot.manipulator;

public enum PivotState {
  STOW(0, 0),
  TEST1(0.2, 0),
  TEST2(0.4, 0);

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
