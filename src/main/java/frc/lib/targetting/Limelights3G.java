package frc.lib.targetting;

import java.util.ArrayList;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import frc.robot.LimelightHelpers;

/** Set of limelight 3Gs */
public class Limelights3G implements Limelights {

  private ArrayList<String> limelights;

  public Limelights3G() {
    limelights = new ArrayList<>();
  }
  
  @Override
  public void addLimelights(String... names) {
    for (String name : names) {
      limelights.add(name);
    }
  }

  @Override
  public void addVisionMeasurements(SwerveDrivePoseEstimator poseEstimator) {
    for (String limelight : limelights) {
      LimelightHelpers.PoseEstimate measurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelight);

      if (measurement.tagCount >= 2) {
        poseEstimator.addVisionMeasurement(
          measurement.pose, 
          measurement.timestampSeconds);
      }
    }
  }
}
