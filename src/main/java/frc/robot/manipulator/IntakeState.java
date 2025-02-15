package frc.robot.manipulator;

public enum IntakeState {
  STOP(0),
  CORALIN(30),
  CORALSCORIN(-30),
  FAST(100);

  private final double velRotationsPerSec;

  IntakeState(double velRotationsPerSec) {
    this.velRotationsPerSec = velRotationsPerSec;
  }

  public double getVelRotationsPerSec() {
    return velRotationsPerSec;
  }
}
