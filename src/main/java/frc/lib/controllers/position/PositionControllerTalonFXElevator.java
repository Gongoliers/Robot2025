package frc.lib.controllers.position;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;

/** Position controller implementation for elevator driven by 2 TalonFX motors */
public class PositionControllerTalonFXElevator implements PositionController {

  /** Mechanism config */
  private final MechanismConfig config;

  /** Leader motor */
  private final TalonFX leader;

  /** Follower motor */
  private final TalonFX follower;

  /** Position of setpoint */
  private Angle setpointPosition;

  /** Velocity of setpoint */
  private AngularVelocity setpointVelocity;

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

  /** PID controller */
  private final PIDController feedback;

  /** Elevator feedforward */
  private final ElevatorFeedforward feedforward;

  /** Voltage request */
  private final VoltageOut voltage;

  /** Position offset (used to re zero the elevator without messing with hardware zero) */
  private Angle positionOffset;

  /** True if voltage is manually set */
  private boolean voltageSet;

  /** Manually set voltage */
  private Voltage setVoltage;

  public PositionControllerTalonFXElevator(
      MechanismConfig config,
      CAN leaderCAN,
      CAN followerCAN,
      boolean invertFollower) {

    // Set config
    this.config = config;

    // Initialize hardware
    leader = new TalonFX(leaderCAN.id(), leaderCAN.bus());
    follower = new TalonFX(followerCAN.id(), followerCAN.bus());

    // Get status signals
    position = leader.getPosition();
    velocity = leader.getVelocity();
    acceleration = leader.getAcceleration();
    motorVoltage = leader.getMotorVoltage();
    statorCurrent = leader.getStatorCurrent();
    supplyCurrent = leader.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity, acceleration, motorVoltage, statorCurrent, supplyCurrent);
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();

    // Set up feedback and feedforward
    feedback = config.feedbackControllerConfig().createPIDController();
    feedforward = config.feedforwardControllerConfig().createElevatorFeedforward();

    // Initialize other variables
    setpointPosition = Rotations.of(0.0);
    setpointVelocity = RotationsPerSecond.of(0.0);
    voltage = new VoltageOut(0.0);
    positionOffset = Rotations.of(0);
    voltageSet = false;
    setVoltage = Volts.of(0.0);

    follower.setControl(new Follower(leaderCAN.id(), invertFollower));

    // Configure hardware
    configure();
  }

  @Override
  public void configure() {
    TalonFXConfigurator leaderConfigurator = leader.getConfigurator();
    TalonFXConfigurator followerConfigurator = follower.getConfigurator();

    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(config.motorConfig().statorCurrentLimit())
        .withSupplyCurrentLimit(config.motorConfig().supplyCurrentLimit()))
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(config.motorConfig().ccwPositive() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
        .withNeutralMode(config.motorConfig().neutralBrake() ? NeutralModeValue.Brake : NeutralModeValue.Coast))
      .withFeedback(new FeedbackConfigs()
        .withRotorToSensorRatio(config.motorConfig().motorToMechRatio()));

    leaderConfigurator.apply(motorConfiguration);
    followerConfigurator.apply(motorConfiguration);
  }

  @Override
  public void getUpdatedVals(PositionControllerValues values) {
    BaseStatusSignal.refreshAll(position, velocity, acceleration, motorVoltage, statorCurrent, supplyCurrent);

    values.position = position.getValue().plus(positionOffset);
    values.velocity = velocity.getValue();
    values.acceleration = acceleration.getValue();
    values.motorVoltage = motorVoltage.getValue();
    values.statorCurrent = statorCurrent.getValue();
    values.supplyCurrent = supplyCurrent.getValue();
  }

  @Override
  public void setPosition(Angle newPos) {
    positionOffset = newPos.minus(position.getValue().plus(positionOffset));
  }

  @Override
  public void setSetpoint(Angle pos, AngularVelocity vel) {
    setpointPosition = pos;
    setpointVelocity = vel;
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
      leader.setControl(voltage.withOutput(setVoltage));
    } else {
      Angle motorPosition = position.getValue().plus(positionOffset);

      double feedbackVolts = feedback.calculate(motorPosition.in(Radians), setpointPosition.in(Radians));
      double feedforwardVolts = feedforward.calculate(setpointPosition.in(Radians), setpointVelocity.in(RadiansPerSecond));

      leader.setControl(voltage.withOutput(feedforwardVolts + feedbackVolts));
    }
  }
}
