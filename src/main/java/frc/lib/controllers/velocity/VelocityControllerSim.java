package frc.lib.controllers.velocity;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotConstants;

/** Simulated velocity controller */
public class VelocityControllerSim implements VelocityController {

  private Angle position;
  private AngularVelocity velocity;

  public VelocityControllerSim() {
    position = Rotations.of(0.0);
    velocity = RotationsPerSecond.of(0.0);
  } 

  @Override
  public void configure() {}

  @Override
  public void getUpdatedVals(VelocityControllerValues values) {
    values.position = position;
    values.velocity = velocity;
  }

  @Override
  public void setSetpoint(AngularVelocity velocity) {
    this.velocity = velocity;
  }

  @Override
  public void periodic() {
    position = position.plus(velocity.times(Seconds.of(RobotConstants.PERIODIC_DURATION)));
  }
}
