package frc.lib.targetting;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Robot;
import frc.robot.RobotConstants;

/** Gets field target positions with compensation for current alliance */
public class FieldTargetSupplier {
  
  /** Get translation of the center of the reef */
  public static Translation2d getReefCenter() {
    return (Robot.isRedAlliance())
      ? new Translation2d(13.0, 4.0)
      : new Translation2d(4.5, 4.0);
  }

  /** 
   * Gets angle from the center of the reef to the center of the robot
   * 
   * @param robotPose current robot pose
   * @return angle from the center of th reef to the center of the robot
   */
  public static Rotation2d getAngleToReef(Pose2d robotPose) {
    return robotPose.getTranslation().minus(getReefCenter()).getAngle();
  }

  /**
   * Gets closest face of reef from robot pose
   * 
   * @param robotPose current robot pose
   * @return index (can be negative) of the closest reef face
   */
  public static int getReefFace(Pose2d robotPose) {
    return (int) ((getAngleToReef(robotPose).getDegrees() + 390) / 60) - 6;
  }

  /**
   * Gets angle of the line normal to the closest reef face
   * 
   * @param robotPose current robot pose
   * @return angle of the line normal to the closest reef face
   */
  public static Rotation2d getReefFaceNormal(Pose2d robotPose) {
    return Rotation2d.fromDegrees(getReefFace(robotPose)*60);
  }

  /**
   * Gets the position on the field of closest reef target with some offset to leave room for bumpers or room for the manipulator to move or something
   * 
   * @param robotPose current robot pose
   * @param target reef target (left right or center)
   * @param safeDistance distance (in meters) to keep between the target and robot
   * @return position of the field of closest reef target with some offset to leave room for bumpers or room for the manipulator to move or something
   */
  public static Translation2d getSafeTranslation(Pose2d robotPose, ReefTarget target, double safeDistance) {
    Rotation2d reefFaceNormal = getReefFaceNormal(robotPose);
    Translation2d offsetTranslation = new Translation2d(
      target.getForwardOffset() + RobotConstants.CHASSIS_SIDE_LENGTH*0.0254/2 + safeDistance,
      target.getHorizontalOffset())
      .rotateBy(reefFaceNormal);

    return getReefCenter().plus(offsetTranslation);
  }
}
