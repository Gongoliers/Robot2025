package frc.robot;

import java.util.EnumSet;
import java.util.Set;

/** Global constants for the robot */
public class RobotConstants {
  
  /** Number of robot periodic calls per second */
  public static final double PERIODIC_RATE = 50;

  /** Duration of each robot periodic call in seconds */
  public static final double PERIODIC_DURATION = 1.0/PERIODIC_RATE;

  /** Subsystems */
  public enum Subsystem {
    SWERVE,
    ODOMETRY,
    LIMELIGHT,
    ELEVATOR,
  }

  /** Enabled subsystems */
  public static final Set<Subsystem> ENABLED_SUBSYSTEMS =
    EnumSet.of(
      Subsystem.SWERVE, Subsystem.ODOMETRY);
}