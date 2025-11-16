package frc.lib;

import frc.robot.Multithreader;

/**
 * Subsystem base class with some extra required functions and a fastPeriodic function that
 * automatically registers itself to the Multithreader
 */
public abstract class MultithreadedSubsystem extends Subsystem implements Multithreaded {

  /** Constructor that reigsters multithreaded subsystem to multithreader */
  public MultithreadedSubsystem() {
    Multithreader.getInstance().registerMultithreaded(this);
  }
}
