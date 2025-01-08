package frc.lib.sensors;

import java.util.function.DoubleSupplier;

import frc.robot.RobotConstants;

/** Simulated gyroscope */
public class GyroscopeSim implements Gyroscope {
  
  private final DoubleSupplier yawVelRotationsPerSec;

  private double yawRotations = 0.0;

  public GyroscopeSim(DoubleSupplier yawVelRotationsPerSec) {
    this.yawVelRotationsPerSec = yawVelRotationsPerSec;
  }

  @Override
  public void configure() {}

  @Override
  public void getUpdatedVals(GyroscopeValues values) {
    values.yawRotations = yawRotations;
    values.yawVelRotationsPerSec = yawVelRotationsPerSec.getAsDouble();
  }

  @Override
  public void setYaw(double yawRotations) {
    this.yawRotations = yawRotations;
  }

  @Override
  public void periodic() {
    yawRotations += yawVelRotationsPerSec.getAsDouble() * RobotConstants.PERIODIC_DURATION;
  }
}
