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

  private boolean voltageSet;
  private Voltage setVoltage;

  /** Initialize simulated position controller */
  public PositionControllerSim() {
    position = Rotations.of(0.0);
    velocity = RotationsPerSecond.of(0.0);
    voltageSet = false;
    setVoltage = Volts.of(0.0);
  }

  @Override
  public void configure() {}

  @Override
  public void getUpdatedVals(PositionControllerValues values) {
    values.position.mut_replace(position);
    values.velocity.mut_replace(velocity);
    values.motorVoltage.mut_replace((voltageSet) ? setVoltage : Volts.of(0.0));
  }

  @Override
  public void setPosition(Angle newPos) {
    position = newPos;
  }

  @Override
  public void setSetpoint(Angle pos, AngularVelocity vel) {
    position = pos;
    velocity = vel;
  }

  @Override
  public void setVoltage(Voltage volts) {
    setVoltage = volts;
    voltageSet = true;
  }

  @Override
  public void clearVoltage() {
    voltageSet = false;
  }

  @Override
  public void periodic() {
    if (voltageSet) {
      position = position.plus(Rotations.of(setVoltage.in(Volts)*2));
    }
  }
}
