package frc.lib;

/** Handles telemetry with Shuffleboard */
public class Telemetry {

  public static void initializeTabs(Subsystem... subsystems) {
    for (Subsystem subsystem : subsystems) {
      subsystem.initializeTab();
    }
  }
}
