package frc.lib.targetting;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.LimelightHelpers;

/**
 * Basic interface for limelights
 * 
 * Outlines required functions and variables all limelights have to make implementing new types
 * of limelights in the future much simpler
 */
public interface Limelights {
  
  /** 
   * Adds a limelight
   * 
   * @param name name of limelight
   * @param forward forward offset in meters
   * @param side side offset in meters
   * @param up up offset in meters
   * @param roll roll offset in degrees
   * @param pitch pitch offset in degrees
   * @param yaw yaw offset in degrees
   */
  public void addLimelight(String name, double forward, double side, double up, double roll, double pitch, double yaw);

  /**
   * Adds vision measurements to a swerve pose estimator
   * 
   * @param poseEstimator swerve drive pose estimator to add vision measurements to
   * @param yaw rotation representing the current yaw of the robot
   */
  public void addVisionMeasurements(SwerveDrivePoseEstimator poseEstimator, Rotation2d yaw);

  /**
   * Gets a raw vision measurement from some limelight
   * 
   * @param cameraName name of limelight
   * @return a raw vision measurement from some limelight
   */
  public LimelightHelpers.PoseEstimate getVisionMeasurement(String cameraName);
}
