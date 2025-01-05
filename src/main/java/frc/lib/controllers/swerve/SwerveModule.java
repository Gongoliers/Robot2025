package frc.lib.controllers.swerve;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Basic interface for swerve modules
 * 
 * Outlines required functions and variables all swerve modules have to make implementing new types
 * of swerve modules in the future much simpler
 */
public interface SwerveModule {
  
  /**
   * Returns the swerve module state
   * 
   * @return the swerve module state
   */
  public SwerveModuleState getState();

  /**
   * Returns the swerve module target state
   * 
   * @return the swerve module target state
   */
  public SwerveModuleState getSetpoint();

  /**
   * Sets the swerve module target state
   * 
   * @param setpoint the swerve module target state
   * @param lazy if true, optimize the swerve setpoint
   */
  public void setSetpoint(SwerveModuleState setpoint, boolean lazy);

  /**
   * Returns the swerve module's current position
   * 
   * @return the swerve module's current position
   */
  public SwerveModulePosition getPosition();
}
