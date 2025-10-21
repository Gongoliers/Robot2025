package frc.lib.motors;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MomentOfInertia;
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

  /** System plant */
  private final LinearSystem<N1, N1, N1> plant;

  /** DC motor sim */
  private final DCMotor motor;

  /** Position of flywhell (found by integrating sim velocity) */
  private Angle position;

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
    plant = LinearSystemId.createFlywheelSystem(motor, moi.in(KilogramSquareMeters), config.rotorToSensorRatio()*config.sensorToMechRatio());
    sim = new FlywheelSim(plant, motor);
    this.kS = kS;
  }

  @Override
  public void setVoltage(Voltage voltage) {
    double volts = voltage.in(Volts);
    sim.setInputVoltage(volts - Math.copySign(MathUtil.clamp(Math.abs(volts), 0, kS), volts));
  }

  @Override
  public void getUpdatedValues(MotorValues values, Time dt) {
    sim.update(dt.in(Seconds));
    position = position.plus(sim.getAngularVelocity().times(dt));

    values.position = position;
    values.velocity = sim.getAngularVelocity();
    values.acceleration = sim.getAngularAcceleration();
    values.motorVoltage = Volts.of(sim.getInputVoltage());
    values.supplyVoltage = Volts.of(sim.getInputVoltage());
    values.statorCurrent = Amps.of(0.0); //TODO not sure how to actually calculate this
    values.supplyCurrent = Amps.of(sim.getCurrentDrawAmps()); //TODO not sure if this represents supply or stator current
  }

  @Override
  public boolean configure() {
    return true;
  }
}
