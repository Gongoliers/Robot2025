package frc.lib.targetting;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;

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
   */
  public void addLimelights(String... names);

  /**
   * Adds vision measurements to a swerve pose estimator
   * 
   * @param poseEstimator swerve drive pose estimator to add vision measurements to
   */
  public void addVisionMeasurements(SwerveDrivePoseEstimator poseEstimator);
}
