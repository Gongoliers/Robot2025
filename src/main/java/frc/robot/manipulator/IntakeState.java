package frc.robot.manipulator;

public enum IntakeState {
  STOP(0),
  CORALIN(22),
  CORALOUT(-22),
  FAST(100);

  private final double velRotationsPerSec;

  IntakeState(double velRotationsPerSec) {
    this.velRotationsPerSec = velRotationsPerSec;
  }

  public double getVelRotationsPerSec() {
    return velRotationsPerSec;
  }
}
