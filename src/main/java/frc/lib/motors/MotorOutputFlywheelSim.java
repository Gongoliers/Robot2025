package frc.lib.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.lib.configs.MotorConfig;

/** Motor output implementation for a simulated flywheel */
public class MotorOutputFlywheelSim implements MotorOutput{

  /** Motor config used for configuration and some constants */
  private final MotorConfig config;
  
  /** Flywheel simulation */
  private final FlywheelSim sim;

  /** DC motor sim */
  private final DCMotor motor;

  /** Position of flywhell (found by integrating sim velocity) */
  private MutAngle position;

  /** Voltage to overcome static friction */
  private final double kS;

  /**
   * Motor output with flywheel simulation constructor
   * 
   * @param config motor config
   * @param motor DC motor sim
   * @param moi moment of inertia of flywheel
   * @param kS voltage to overcome static friction
   */
  public MotorOutputFlywheelSim(
      MotorConfig config,
      DCMotor motor,
      MomentOfInertia moi,
      double kS) {

    // Set config
    this.config = config;
      
    // Set up sim
    this.motor = motor;
    LinearSystem<N1, N1, N1> plant = LinearSystemId.createFlywheelSystem(motor, moi.in(KilogramSquareMeters), config.rotorToSensorRatio()*config.sensorToMechRatio());
    sim = new FlywheelSim(plant, motor);
    this.kS = kS;
    this.position = Rotations.mutable(0.0);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    sim.setInputVoltage(calculateEffectiveVoltage(voltage.in(Volts)));
  }

  /**
   * Calculates effective voltage applied to sim (voltage will stay locked at zero until voltage applied by setVoltage is greater than magnitude kS)
   * 
   * @param voltage voltage applied by setVoltage
   * @return effective voltage applied to sim
   */
  private double calculateEffectiveVoltage(double voltage) {
    return voltage - Math.copySign(MathUtil.clamp(Math.abs(voltage), 0, kS), voltage);
  }

  @Override
  public void updateValues(MotorValues values, Time dt) {
    sim.update(dt.in(Seconds));
    position.mut_plus(sim.getAngularVelocity().times(dt));

    values.position.mut_replace(position);
    values.velocity.mut_replace(sim.getAngularVelocity());
    values.acceleration.mut_replace(sim.getAngularAcceleration());
    values.motorVoltage.mut_replace(sim.getInputVoltage(), Volts);
    values.supplyVoltage.mut_replace(sim.getInputVoltage(), Volts);
    values.statorCurrent.mut_replace(0, Amps); //TODO not sure how to actually calculate this
    values.supplyCurrent.mut_replace(sim.getCurrentDrawAmps(), Amps); //TODO not sure if this represents supply or stator current
  }

  @Override
  public boolean configure() {
    return true;
  }
}
