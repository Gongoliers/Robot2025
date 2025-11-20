package frc.lib.motors;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class MotorOutputSim implements MotorOutput {

  private final DCMotorSim sim;

  private final MutAngle positionOffset;

  /**
   * Creates a simulated motor system.
   *
   * @param sim The motor system to simulate.
   */
  public MotorOutputSim(DCMotorSim sim) {
    this.sim = sim;

    positionOffset = Rotations.mutable(0.0);
  }

  /**
   * Creates a simulated motor system.
   *
   * @param kV The coefficient relating voltage to velocity.
   * @param kA The coefficient relating voltage to acceleration.
   * @param gearbox The type and number of motors in the system.
   */
  public MotorOutputSim(
      Measure<PerUnit<VoltageUnit, AngularVelocityUnit>> kV,
      Measure<PerUnit<VoltageUnit, AngularAccelerationUnit>> kA,
      DCMotor gearbox) {
    this(
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(
                kV.in(Volts.per(RadiansPerSecond)), kA.in(Volts.per(RadiansPerSecondPerSecond))),
            gearbox));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    sim.setInputVoltage(voltage.in(Volts));
  }

  @Override
  public void setPosition(Angle newPosition) {
    positionOffset.mut_replace(newPosition.minus(Radians.of(sim.getAngularPositionRad())));
  }

  @Override
  public void updateValues(MotorValues values, Time dt) {
    sim.update(dt.in(Seconds));

    values.position.mut_replace(sim.getAngularPositionRad() + positionOffset.in(Radians), Radians);
    values.velocity.mut_replace(sim.getAngularVelocityRadPerSec(), RadiansPerSecond);
    values.acceleration.mut_replace(
        sim.getAngularAccelerationRadPerSecSq(), RadiansPerSecondPerSecond);

    double motorVoltage = sim.getInputVoltage();
    double supplyVoltage = RobotController.getBatteryVoltage();
    double dutyCycle = motorVoltage / supplyVoltage;
    double statorCurrent = sim.getCurrentDrawAmps();
    double supplyCurrent = statorCurrent * dutyCycle;

    values.motorVoltage.mut_replace(motorVoltage, Volts);
    values.supplyVoltage.mut_replace(supplyVoltage, Volts);
    values.statorCurrent.mut_replace(statorCurrent, Amps);
    values.supplyCurrent.mut_replace(supplyCurrent, Amps);
  }

  @Override
  public boolean configure() {
    return true;
  }
}
