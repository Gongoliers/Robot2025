package frc.lib.motors;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import java.util.function.Function;

public class MotorOutputSim implements MotorOutput {

  private final DCMotorSim sim;

  private final Voltage kS;

  private final Function<Angle, Voltage> kG;

  private final MutAngle position;

  private final MutVoltage voltage;

  /**
   * Creates a simulated motor system with a constant static friction voltage loss and a possibly
   * variable gravity voltage loss.
   *
   * @param sim The motor system to simulate.
   * @param kS The voltage loss due to static friction.
   * @param kG The voltage loss due to gravity, dependent on motor position.
   */
  public MotorOutputSim(DCMotorSim sim, Voltage kS, Function<Angle, Voltage> kG) {
    this.sim = sim;
    this.kS = kS;
    this.kG = kG;
    this.position = Radians.mutable(0);
    this.voltage = Volts.mutable(0);
  }

  /**
   * Creates a simulated motor system with a constant static friction voltage loss and a constant
   * gravity voltage loss.
   *
   * @param sim The motor system to simulate.
   * @param kS The voltage loss due to static friction.
   * @param kG The voltage loss due to gravity, independent of motor position.
   */
  public MotorOutputSim(DCMotorSim sim, Voltage kS, Voltage kG) {
    this(sim, kS, position -> kG);
  }

  /**
   * Creates a simulated motor system with a constant static friction voltage loss and no gravity
   * voltage loss.
   *
   * @param sim The motor system to simulate.
   * @param kS The voltage loss due to static friction.
   */
  public MotorOutputSim(DCMotorSim sim, Voltage kS) {
    this(sim, kS, Volts.zero());
  }

  /**
   * Creates a simulated motor system with no static friction voltage loss and no gravity voltage
   * loss.
   *
   * @param sim The motor system to simulate.
   */
  public MotorOutputSim(DCMotorSim sim) {
    this(sim, Volts.zero());
  }

  @Override
  public void setVoltage(Voltage voltage) {
    this.voltage.mut_replace(voltage);
    double volts = voltage.in(Volts);
    double effectiveVoltage = volts - calculateVoltageLoss(volts);
    sim.setInputVoltage(effectiveVoltage);
  }

  /**
   * Calculates the voltage losses due to static friction and gravity.
   *
   * @param voltage The voltage being applied to the system, prior to any losses.
   * @return The voltage loss due to static friction and gravity.
   */
  private double calculateVoltageLoss(double voltage) {
    double kS = this.kS.in(Volts);
    double kG = this.kG.apply(position).in(Volts);
    if (Math.abs(voltage) < kS) {
      return kG;
    }
    return Math.copySign(kS, voltage) + kG;
  }

  @Override
  public void updateValues(MotorValues values, Time dt) {
    sim.update(dt.in(Seconds));
    this.position.mut_replace(sim.getAngularPositionRad(), Radians);

    double motorVoltage = voltage.in(Volts);
    double supplyVoltage = RobotController.getBatteryVoltage();
    double dutyCycle = motorVoltage / supplyVoltage;
    double statorCurrent = sim.getCurrentDrawAmps();
    double supplyCurrent = statorCurrent * dutyCycle;

    values.position.mut_replace(position);
    values.velocity.mut_replace(sim.getAngularVelocityRadPerSec(), RadiansPerSecond);
    values.acceleration.mut_replace(
        sim.getAngularAccelerationRadPerSecSq(), RadiansPerSecondPerSecond);
    values.motorVoltage.mut_replace(voltage);
    values.supplyVoltage.mut_replace(supplyVoltage, Volts);
    values.statorCurrent.mut_replace(statorCurrent, Amps);
    values.supplyCurrent.mut_replace(supplyCurrent, Amps);
  }

  @Override
  public boolean configure() {
    return true;
  }
}
