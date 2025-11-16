package frc.lib;

/**
 * Interface that gives a class a fastPeriodic method to be called by the multithreader at a rate
 * faster than the standard 50hz
 */
public interface Multithreaded {

  /** Called periodically by the multithreader at a rate faster than the standard 50hz */
  public abstract void fastPeriodic();
}
