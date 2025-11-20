package frc.lib.controllers.velocity;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.RobotConstants;

/** Simulated velocity controller */
public class VelocityControllerSim implements VelocityController {

  private Angle position;
  private AngularVelocity velocity;

  private boolean voltageSet;
  private Voltage setVoltage;

  public VelocityControllerSim() {
    position = Rotations.of(0.0);
    velocity = RotationsPerSecond.of(0.0);

    voltageSet = false;
    setVoltage = Volts.of(0.0);
  }

  @Override
  public void configure() {}

  @Override
  public void getUpdatedVals(VelocityControllerValues values) {
    values.position.mut_replace(position);
    values.velocity.mut_replace(velocity);

    if (voltageSet) {
      values.motorVoltage.mut_replace(setVoltage);
    } else {
      values.motorVoltage.mut_replace(0.0, Volts);
    }
  }

  @Override
  public void setSetpoint(AngularVelocity velocity) {
    if (!voltageSet) {
      this.velocity = velocity;
    }
  }

  @Override
  public void setVoltage(Voltage volts) {
    setVoltage = volts;
    voltageSet = true;

    this.velocity = RotationsPerSecond.of(volts.in(Volts) * 1);
  }

  @Override
  public void clearVoltage() {
    voltageSet = false;
  }

  @Override
  public void periodic() {
    position = position.plus(velocity.times(Seconds.of(RobotConstants.PERIODIC_DURATION)));
  }
}
