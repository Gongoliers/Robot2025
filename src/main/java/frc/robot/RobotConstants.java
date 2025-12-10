package frc.robot;

import java.util.EnumSet;
import java.util.Set;

/** Global constants for the robot */
public class RobotConstants {

  /** Number of robot periodic calls per second */
  public static final double PERIODIC_RATE = 50;

  /** Duration of each robot periodic call in seconds */
  public static final double PERIODIC_DURATION = 1.0 / PERIODIC_RATE;

  /** Number of fast peridic calls per second */
  public static final double FAST_PERIODIC_RATE = 100;

  /** Duration of each fast periodic call in seconds */
  public static final double FAST_PERIODIC_DURATION = 1.0 / FAST_PERIODIC_RATE;

  /** Side lengths of robot chassis */
  public static final double CHASSIS_SIDE_LENGTH = 24;

  /** Subsystems */
  public enum Subsystem {
    SWERVE,
    ODOMETRY,
    LIMELIGHT,
    ELEVATOR,
    PIVOT,
    ENDGAME,
    INTAKE,
    SUPERSTRUCTURE,
    AUTO,
  }

  /** Enabled subsystems */
  public static final Set<Subsystem> ENABLED_SUBSYSTEMS = EnumSet.of(Subsystem.SWERVE);
}
