package frc.robot.manipulator;

import frc.lib.Subsystem;

/** Manipulator subsystem */
public class Manipulator extends Subsystem {
  
  /** Manipulator singleton */
  private static Manipulator instance = null;

  /** Motors */
  private final 

  /** Initializes the Manipulator subsystem and configures hardware */
  private Manipulator() {

  }

  /** Gets manipulator subsystem instance if there is one, and creates one if there isn't */
  public static Manipulator getInstance() {
    if (instance == null) {
      instance = new Manipulator();
    }

    return instance;
  }

  @Override
  public void initializeTab() {

  }

  @Override
  public void periodic() {

  }
}
