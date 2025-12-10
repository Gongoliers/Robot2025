package frc.robot.drive;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.List;
import java.util.Optional;
import java.util.function.Predicate;

public class AprilTagTargets {

  private static Pose2d flip(Pose2d pose) {
    return pose.rotateAround(pose.getTranslation(), Rotation2d.k180deg);
  }

  private static Pose2d facingTag(AprilTag tag) {
    return flip(tag.pose.toPose2d());
  }

  public static boolean isBlueReefTag(AprilTag tag) {
    return 17 <= tag.ID && tag.ID <= 22;
  }

  public static boolean isRedReefTag(AprilTag tag) {
    return 6 <= tag.ID && tag.ID <= 11;
  }

  public static List<Pose2d> targets(AprilTagFieldLayout layout, Predicate<AprilTag> filter) {
    return layout.getTags().stream().filter(filter).map(AprilTagTargets::facingTag).toList();
  }

  public static List<Pose2d> blueReefTargets(AprilTagFieldLayout layout) {
    return targets(layout, AprilTagTargets::isBlueReefTag);
  }

  public static List<Pose2d> redReefTargets(AprilTagFieldLayout layout) {
    return targets(layout, AprilTagTargets::isRedReefTag);
  }

  public static List<Pose2d> reefTargets(
      AprilTagFieldLayout layout, Optional<DriverStation.Alliance> alliance) {
    return alliance
        .map(
            value ->
                switch (value) {
                  case Blue -> blueReefTargets(layout);
                  case Red -> redReefTargets(layout);
                })
        .orElseGet(List::of);
  }
}
