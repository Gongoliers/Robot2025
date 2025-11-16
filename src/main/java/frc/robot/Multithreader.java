package frc.robot;

import frc.lib.Multithreaded;
import java.util.HashSet;
import java.util.Set;

/** Class that handles calling fastPeriodic methods in multithreaded classes */
public class Multithreader extends Thread {

  /** Multithreader instance */
  private static Multithreader instance = null;

  /** Target interval between calls of fastPeriodic in nanoseconds */
  private final long nanoTime = (long) (RobotConstants.FAST_PERIODIC_DURATION * 1000000000);

  /** Set of objects with fastPeriodic method to be called */
  private Set<Multithreaded> multithreadeds = new HashSet<Multithreaded>();

  /**
   * Gets multithreader instance
   *
   * @return multithreader instance
   */
  public static Multithreader getInstance() {
    if (instance == null) {
      instance = new Multithreader();
    }

    return instance;
  }

  private Multithreader() {
    setName("Multithreader");
    setDaemon(true);
  }

  /**
   * Register a multithreaded class to have their fastPeriodic function called each periodic
   * duration
   *
   * @param multithreaded multithreaded class to register
   */
  public void registerMultithreaded(Multithreaded multithreaded) {
    multithreadeds.add(multithreaded);
  }

  @Override
  public void run() {
    while (true) {
      long startTime = System.nanoTime();

      fastPeriodic();

      // Pause the thread just long enough for every loop of the thread to last exactly as long as
      // expected
      long sleepTime = nanoTime - System.nanoTime() + startTime;

      if (sleepTime > 0) {
        try {
          Thread.sleep(sleepTime / 1000000, (int) (sleepTime % 1000000));
        } catch (InterruptedException e) {
          System.out.println("Multithreader interrupted");
          break;
        }
      } else {
        // If thread takes longer to execute tasks than expected, don't pause, and log how much it
        // has overrun by
        System.out.println("SpeedThread overran by " + (-sleepTime / 1000000) + "ms");
      }
    }
  }

  private void fastPeriodic() {
    for (Multithreaded multithreaded : multithreadeds) {
      multithreaded.fastPeriodic();
    }
  }
}
