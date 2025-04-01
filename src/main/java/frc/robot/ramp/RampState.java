package frc.robot.ramp;

public enum RampState {
  STOP(0),
  INTAKESLOW(10),
  INTAKEFAST(25),
  NONE(999);

  private final double velRotationsPerSec;

  RampState(double velRotationsPerSec) {
    this.velRotationsPerSec = velRotationsPerSec;
  }

  /** 
   * Gets the velocity in rotations per second of the ramp state
   * 
   * @return the velocity in rotations per second of the ramp state
   */
  public double getVelRotationsPerSec() {
    return velRotationsPerSec;
  }
}