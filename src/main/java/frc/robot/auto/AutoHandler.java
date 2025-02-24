package frc.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;

/** Keeps track of whether or not the robot is being controlled autonomously right now */
public class AutoHandler {
  
  /** True if a pathplanner path or some other autonomous thing is being used in teleop */
  private static boolean isTeleAuto = false;

  /** Set to true if robot is being controlled autonomously in teleop, false otherwise */
  public static void setIsTeleAuto(boolean isTeleAuto) {
    AutoHandler.isTeleAuto = isTeleAuto;
  }

  /** True if robot is being controlled autonomously */
  public static boolean getIsAuto() {
    return isTeleAuto || DriverStation.isAutonomous();
  }
}
