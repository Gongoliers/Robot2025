package frc.lib.controllers.position;

public class EndgamePositionControllerSim implements EndgamePositionController {
  
  /** Configures the position controller */
  public void configure() {}

  /**
   * Get the position controller's updated values and update the provided values class accordingly
   * 
   * @param values values class to be updated
   */
  public void getUpdatedVals(EndgamePositionControllerValues values) {}

  /**
   * Sets the position of the position controller
   * 
   * @param posRotations position in rotations
   */
  public void setPos(double posRotations) {}

  /**
   * Sets the setpoint (target position and velocity) of the controller
   * 
   * @param posRotations target position in rotations
   * @param velRotationsPerSec target velocity in rotations per second
   */
  public void setSetpoint(double posRotations, double velRotationsPerSec) {}

  /**
   * Sets a specific constant voltage
   * 
   * @param voltage set voltage
   */
  public void setVoltage(double voltage) {}

  /** Clears a manually set voltage */
  public void clearSetVoltage() {}

  /** Called every periodic loop */
  public void periodic() {}
}
