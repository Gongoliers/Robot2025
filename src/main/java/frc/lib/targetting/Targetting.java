package frc.lib.targetting;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotConstants;
import frc.robot.auto.AutonomousHandler;

/** Handles targetting on the field */
public class Targetting {

  /** 
   * Gets angle from the center of the reef to the center of the robot
   * 
   * @param robotPose current robot pose
   * @return angle from the center of the reef to the center of the robot
   */
  public static Rotation2d getAngleToReef(Pose2d robotPose) {
    return robotPose.getTranslation().minus(FieldTargetSupplier.getReefCenter()).getAngle();
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
  public static Translation2d getSafeTranslation(Pose2d robotPose, ReefTarget target, double safeDistance) {
    Rotation2d reefFaceNormal = getReefFaceNormal(robotPose);
    Translation2d offsetTranslation = new Translation2d(
      target.getForwardOffset() + RobotConstants.CHASSIS_SIDE_LENGTH*0.0254/2 + safeDistance, // magic number converts to meters
      target.getHorizontalOffset())
      .rotateBy(reefFaceNormal);

    return FieldTargetSupplier.getReefCenter().plus(offsetTranslation);
  }

  public static Command getPathingCommand(Pose2d robotPose, ReefTarget target, double safeDistance) {
    Translation2d targetTranslation = getSafeTranslation(robotPose, target, safeDistance);
    Rotation2d targetRotation = getReefFaceNormal(robotPose).rotateBy(Rotation2d.k180deg);

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
      new Pose2d(robotPose.getTranslation(), robotPose.getTranslation().minus(targetTranslation).getAngle()),
      new Pose2d(targetTranslation, targetRotation));

    List<EventMarker> eventMarkers = Arrays.asList(
      new EventMarker("END PATH", 1.0));

    PathConstraints constraints = new PathConstraints(3, 3, 2 * Math.PI, 4 * Math.PI);

    PathPlannerPath path = new PathPlannerPath(
      waypoints,
      Collections.emptyList(),
      Collections.emptyList(),
      Collections.emptyList(),
      eventMarkers,
      constraints, 
      null, 
      new GoalEndState(0.0, targetRotation),
      false);

    path.preventFlipping = true;

    return AutoBuilder.followPath(path);
  }

  public static Command pathToTarget(Pose2d robotPose, ReefTarget target, double safeDistance) {
    return Commands.runOnce(() -> {
      Command pathingCommand = getPathingCommand(robotPose, target, safeDistance);
    
      AutonomousHandler.setTeleAutonomous(true);
      pathingCommand.schedule();
    })
    .andThen(Commands.waitUntil(() -> !AutonomousHandler.getIsAutonomous()))
    .andThen(Commands.print("ALL DONE!"));
  }
}