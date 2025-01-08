package frc.lib.controllers.position;

public class PositionControllerSim implements PositionController {
  
  private double posRotations = 0.0;

  private double velRotationsPerSec = 0.0;

  @Override
  public void configure() {}

  @Override
  public void getUpdatedVals(PositionControllerValues values) {
    values.posRotations = posRotations;
    values.velRotationsPerSec = velRotationsPerSec;
  }

  @Override
  public void setPos(double posRotations) {
    this.posRotations = posRotations;
  }

  @Override
  public void setSetpoint(double posRotations, double velRotationsPerSec) {
    this.posRotations = posRotations;
    this.velRotationsPerSec = velRotationsPerSec;
  }

  @Override
  public void periodic() {}
}
