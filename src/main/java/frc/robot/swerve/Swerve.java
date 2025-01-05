package frc.robot.swerve;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Swerve subsystem */
public class Swerve extends SubsystemBase {
    
  /** Swerve subsystem singleton */
  private static Swerve instance = null;

  /** Initializes swerve subsystem and configures swerve hardware */
  private Swerve() {

  }

  /**
   * Returns the swerve subsystem instance
   * Creates a new instance if there isn't one yet
   * 
   * @return the swerve subsystem instance
   */
  public static Swerve getInstance() {
    if (instance == null) {
      instance = new Swerve();
    }

    return instance;
  }

  @Override
  public void periodic() {}
}
