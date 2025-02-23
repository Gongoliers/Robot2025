package frc.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;

/** Allows different parts of code to decide if and check if the robot is being controlled autonomously */
public class AutonomousHandler {
  
  /** Is the robot being controlled autonomously OUTSIDE of the autonomous period */
  private static boolean isTeleAutonomous;

  /** Set if the robot is currently controlled autonomously OUTSIDE of the autonomous period */
  public static void setTeleAutonomous(boolean isTeleAutonomous) {
    AutonomousHandler.isTeleAutonomous = isTeleAutonomous;
  }

  /** Get if the robot is currently controlled autonomously */
  public static boolean getIsAutonomous() {
    return DriverStation.isAutonomous() || isTeleAutonomous;
  }
}
