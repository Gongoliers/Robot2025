package frc.robot.auto;

import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.targetting.ReefTarget;

/** Keeps track of whether or not the robot is being controlled autonomously right now */
public class AutoCoordinator {
  
  /** True if a pathplanner path or some other autonomous thing is being used in teleop */
  private static boolean isTeleAuto = false;

  /** Most recently targetting reef target */
  private static ReefTarget recentReefTarget = ReefTarget.LEFT;

  /** Set to true if robot is being controlled autonomously in teleop, false otherwise */
  public static void setIsTeleAuto(boolean isTeleAuto) {
    AutoCoordinator.isTeleAuto = isTeleAuto;
  }

  /** True if robot is being controlled autonomously */
  public static boolean getIsAuto() {
    return isTeleAuto || DriverStation.isAutonomous();
  }

  /** Set most recent reef target */
  public static void setRecentReefTarget(ReefTarget reefTarget) {
    AutoCoordinator.recentReefTarget = reefTarget;
  }

  /** Get most recent reef target */
  public static ReefTarget getRecentReefTarget() {
    return AutoCoordinator.recentReefTarget;
  }
}
