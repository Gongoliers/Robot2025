package frc.lib.motors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.CAN;
import frc.lib.configs.MotorConfig;

/** Motor output implementation for two TalonFX controlled motors */
public class MotorOutputTalonFX2 implements MotorOutput {

  /** Motor config */
  private final MotorConfig config;

  /** Leader motor */
  private final TalonFX leader;

  /** Follower motor */
  private final TalonFX follower;

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Voltage> supplyVoltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

  /** Voltage control request object */
  private VoltageOut voltage = new VoltageOut(0.0);

  /** 
   * Motor output constructor
   * 
   * @param config motor config used to configure motor
   * @param leaderCAN CAN id and bus for leader motor
   * @param followerCAN CAN id and bus for follower motor
   */
  public MotorOutputTalonFX2(
      MotorConfig config,
      CAN leaderCAN,
      CAN followerCAN,
      boolean invertFollower) {
    
    // set config
    this.config = config;

    // create hardware and status signals
    leader = new TalonFX(leaderCAN.id(), leaderCAN.bus());
    follower = new TalonFX(followerCAN.id(), followerCAN.bus());

    follower.setControl(new Follower(leaderCAN.id(), invertFollower));

    position = leader.getPosition();
    velocity = leader.getVelocity();
    acceleration = leader.getAcceleration();
    motorVoltage = leader.getMotorVoltage();
    supplyVoltage = leader.getSupplyVoltage();
    statorCurrent = leader.getStatorCurrent();
    supplyCurrent = leader.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity, acceleration, motorVoltage, supplyVoltage, statorCurrent, supplyCurrent);
    ParentDevice.optimizeBusUtilizationForAll(leader, follower);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    leader.setControl(this.voltage.withOutput(voltage));
  }

  @Override
  public void updateValues(MotorValues values, Time dt) {
    BaseStatusSignal.refreshAll(position, velocity, acceleration, motorVoltage, supplyVoltage, statorCurrent, supplyCurrent);

    values.position = position.getValue();
    values.velocity = velocity.getValue();
    values.acceleration = acceleration.getValue();
    values.motorVoltage = motorVoltage.getValue();
    values.supplyVoltage = supplyVoltage.getValue();
    values.statorCurrent = statorCurrent.getValue();
    values.supplyCurrent = supplyCurrent.getValue();
  }

  @Override
  public boolean configure() {
    // TODO Allow failed configurations to return false
    TalonFXConfigurator leaderConfigurator = leader.getConfigurator();
    TalonFXConfigurator followerConfigurator = follower.getConfigurator();

    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(config.statorCurrentLimit())
        .withSupplyCurrentLimit(config.supplyCurrentLimit()))
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(config.ccwPositive() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
        .withNeutralMode(config.neutralBrake() ? NeutralModeValue.Brake : NeutralModeValue.Coast))
      .withFeedback(new FeedbackConfigs()
        .withRotorToSensorRatio(config.rotorToSensorRatio())
        .withSensorToMechanismRatio(config.sensorToMechRatio()));

    leaderConfigurator.apply(motorConfiguration);
    followerConfigurator.apply(motorConfiguration);

    return true;
  }
}
