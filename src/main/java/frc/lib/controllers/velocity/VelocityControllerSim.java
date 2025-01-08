package frc.lib.controllers.velocity;

import frc.robot.RobotConstants;

/** Simulated position controller */
public class VelocityControllerSim implements VelocityController {
  
  private double posRotations = 0.0;

  private double velRotationsPerSec = 0.0;

  @Override
  public void configure() {}

  @Override
  public void getUpdatedVals(VelocityControllerValues values) {
    values.posRotations = posRotations;
    values.velRotationsPerSec = velRotationsPerSec;
  }

  @Override
  public void setPos(double posRotations) {
    this.posRotations = posRotations;
  }

  @Override
  public void setSetpoint(double velRotationsPerSec) {
    this.velRotationsPerSec = velRotationsPerSec;
  }

  @Override
  public void periodic() {
    posRotations += velRotationsPerSec * RobotConstants.PERIODIC_DURATION;
  }
}
