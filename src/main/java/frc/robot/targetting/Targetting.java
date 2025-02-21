package frc.robot.targetting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotConstants;

/** Handles targetting on the field */
public class Targetting {
  
  /** Position of the center of the reef */
  private static final Translation2d reefCenter = new Translation2d(4.5, 4);

  /** 
   * Gets angle from the center of the reef to the center of the robot
   * 
   * @param robotPose current robot pose
   * @return angle from the center of the reef to the center of the robot
   */
  public static Rotation2d getAngleToReef(Pose2d robotPose) {
    return robotPose.getTranslation().minus(reefCenter).getAngle();
  }

  /**
   * Get closest face of reef from robot pose
   * 
   * @param robotPose current robot pose
   * @return index (can be negative) of the closest reef face
   */
  public static int getReefFace(Pose2d robotPose) {
    return (int) ((getAngleToReef(robotPose).getDegrees() + 390) / 60) - 6;
  }

  /** 
   * Get angle of line normal to closest reef face
   * 
   * @param robotPose current robot pose
   * @return angle of the line normal to the closest reef face
   */
  public static Rotation2d getReefFaceNormal(Pose2d robotPose) {
    return Rotation2d.fromDegrees(getReefFace(robotPose)*60);
  }

  /**
   * Get the position on the field of closest reef target with some offset to leave room for bumpers or room for the manipulator to move
   * 
   * @param robotPose current robot pose
   * @param target reef  (left right or center)
   * @param safeDistance distance (in meters) to keep between the target position and robot
   * @return position on the field of closest reef target with some offset to leave room for bumpers or room for the manipulator to move
   */
  public static Translation2d getStagedTranslation(Pose2d robotPose, ReefTarget target, double stagingDistance) {
    Rotation2d reefFaceNormal = getReefFaceNormal(robotPose);
    Translation2d offsetTranslation = new Translation2d(
      target.getForwardOffset() + RobotConstants.CHASSIS_SIDE_LENGTH*0.0254/2 + stagingDistance, // magic number converts to meters
      target.getHorizontalOffset())
      .rotateBy(reefFaceNormal);

    return reefCenter.plus(offsetTranslation);
  }
}