package frc.lib.targetting;

import java.util.ArrayList;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.odometry.Odometry;
import frc.robot.LimelightHelpers;

/** Set of limelight 3Gs */
public class Limelights3G implements Limelights {

  /** List of limelight names */
  private ArrayList<String> limelights;

  public Limelights3G() {
    limelights = new ArrayList<>();
  }
  
  @Override
  public void addLimelight(String name, double forward, double side, double up, double roll, double pitch, double yaw) {
    limelights.add(name);
    LimelightHelpers.setCameraPose_RobotSpace(name, forward, side, up, roll, pitch, yaw);
  }

  @Override
  public void addVisionMeasurements(SwerveDrivePoseEstimator poseEstimator, Rotation2d yaw) {
    for (String limelight : limelights) {
      LimelightHelpers.SetRobotOrientation(limelight, yaw.getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelight);

      if (measurement.tagCount >= 1) {
        poseEstimator.addVisionMeasurement(
          measurement.pose,
          measurement.timestampSeconds);
      }
    }
  }

  @Override
  public LimelightHelpers.PoseEstimate getVisionMeasurement(String cameraName) {
    return LimelightHelpers.getBotPoseEstimate_wpiBlue(cameraName);
  }
}
