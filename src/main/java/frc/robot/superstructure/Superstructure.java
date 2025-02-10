package frc.robot.superstructure;

import frc.lib.Subsystem;

/** Superstructure subsystem */
public class Superstructure extends Subsystem {
  
  /** Superstructure subsystem singleton */
  private static Superstructure instance = null;

  /** Initializes superstructure subsystem */
  private Superstructure() {

  }

  /**
   * Gets superstructure subsystem instance if there is one, and creates and returns one if there isn't
   * 
   * @return superstructure subsystem instance
   */
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }

    return instance;
  }

  @Override
  public void periodic() {

  }

  @Override
  public void initializeTab() {
    
  }
}
