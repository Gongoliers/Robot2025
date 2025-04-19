package frc.robot.intake;

public enum IntakeState {
  STOP(0),
  CORALINSLOW(4),
  CORALIN(10),
  CORALOUT(-7),
  CORALOUTSLOW(-4),
  CORALINFAST(32),
  CORALOUTFAST(-32),  
  NONE(999);

  private final double velRotationsPerSec;

  IntakeState(double velRotationsPerSec) {
    this.velRotationsPerSec = velRotationsPerSec;
  }

  /** 
   * Gets the velocity in rotations per second of the intake state
   * 
   * @return the velocity in rotations per second of the intake state
   */
  public double getVelRotationsPerSec() {
    return velRotationsPerSec;
  }
}
