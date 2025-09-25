package frc.lib.controllers.position;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

/** Simulated position controller */
public class PositionControllerSim implements PositionController {
    
  private Angle position;
  private AngularVelocity velocity;
  private Voltage voltage;

  /** Initialize simulated position controller */
  public PositionControllerSim() {
    position = Rotations.of(0.0);
    velocity = RotationsPerSecond.of(0.0);
    voltage = Volts.of(0.0);
  }

  @Override
  public void configure() {}

  @Override
  public void getUpdatedVals(PositionControllerValues values) {
    values.position = position;
    values.velocity = velocity;
    values.motorVoltage = voltage;
  }

  @Override
  public void setPos(Angle newPos) {
    position = newPos;
  }

  @Override
  public void setSetpoint(Angle pos, AngularVelocity vel) {
    position = pos;
    velocity = vel;
  }

  @Override
  public void setVoltage(Voltage volts) {
    voltage = volts;
  }

  @Override
  public void clearVoltage() {
    voltage = Volts.of(0.0);
  }

  @Override
  public void periodic() {}
}
