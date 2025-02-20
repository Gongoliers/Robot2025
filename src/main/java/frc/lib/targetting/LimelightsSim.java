package frc.lib.targetting;

import java.util.ArrayList;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

/** Set of limelight 3Gs */
public class LimelightsSim implements Limelights {

  private ArrayList<String> limelights;

  public LimelightsSim() {
    limelights = new ArrayList<>();
  }
  
  @Override
  public void addLimelight(String name, double forward, double side, double up, double roll, double pitch, double yaw) {
    limelights.add(name);
  }

  @Override
  public void addVisionMeasurements(SwerveDrivePoseEstimator poseEstimator) { }
}