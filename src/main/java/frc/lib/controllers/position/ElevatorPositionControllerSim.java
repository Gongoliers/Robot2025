package frc.lib.controllers.position;

import frc.lib.configs.MechanismConfig;

/** Elevator position controller simulation */
public class ElevatorPositionControllerSim implements ElevatorPositionController {
  private double posMeters = 0.0;
  private double velMetersPerSec = 0.0;

  public ElevatorPositionControllerSim() {}

  @Override
  public void configure() {}

  @Override
  public void getUpdatedVals(ElevatorPositionControllerValues values) {
    values.posMeters = posMeters;
    values.velMetersPerSec = velMetersPerSec;
  }

  @Override
  public double getElevatorPos() {
    return posMeters;
  }

  @Override
  public void setElevatorPos(double posMeters) {
    this.posMeters = posMeters;
  }

  // sim snaps directly to setpoints
  @Override
  public void setSetpoint(double posMeters, double velMetersPerSec) {
    this.posMeters = posMeters;
    this.velMetersPerSec = velMetersPerSec;
  }

  @Override
  public void periodic() {}
}
