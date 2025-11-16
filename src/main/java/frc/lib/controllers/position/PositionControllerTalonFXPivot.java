package frc.lib.controllers.position;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;

/** Position controller for pivot with one TalonFX controlled motor */
public class PositionControllerTalonFXPivot implements PositionController {

  /** Mechanism config */
  private final MechanismConfig config;

  /** TalonFX motor */
  private final TalonFX motor;

  /** Setpoint position */
  private Angle setpointPosition;

  /** Setpoint velocity */
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

  /** Feedforward controller */
  private final ArmFeedforward feedforward;

  /** Voltage conrol request object */
  private final VoltageOut voltage;

  /** Position offset (used to re zero without messing with motor control) */
  private Angle positionOffset;

  /** True if there is a manually set voltage */
  private boolean voltageSet;

  /** If there is a manually set voltage, this is the voltage */
  private Voltage setVoltage;

  public PositionControllerTalonFXPivot(MechanismConfig config, CAN motorCAN) {

    // Set config
    this.config = config;

    // Initialize hardware
    motor = new TalonFX(motorCAN.id(), motorCAN.bus());

    // Get status signals
    position = motor.getPosition();
    velocity = motor.getVelocity();
    acceleration = motor.getAcceleration();
    motorVoltage = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, position, velocity, acceleration, motorVoltage, statorCurrent, supplyCurrent);
    motor.optimizeBusUtilization();

    // Set up feedback and feedforward
    feedback = config.feedbackControllerConfig().createPIDController();
    feedforward = config.feedforwardControllerConfig().createArmFeedforward();

    // Other initialization
    voltage = new VoltageOut(0.0);
    voltageSet = false;
    setVoltage = Volts.of(0.0);
    positionOffset = Rotations.of(0.0);

    // Configure hardware
    configure();
  }

  @Override
  public void configure() {
    TalonFXConfigurator motorConfigurator = motor.getConfigurator();

    TalonFXConfiguration motorConfiguration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(config.motorConfig().statorCurrentLimit())
                    .withSupplyCurrentLimit(config.motorConfig().supplyCurrentLimit()))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        config.motorConfig().ccwPositive()
                            ? InvertedValue.CounterClockwise_Positive
                            : InvertedValue.Clockwise_Positive)
                    .withNeutralMode(
                        config.motorConfig().neutralBrake()
                            ? NeutralModeValue.Brake
                            : NeutralModeValue.Coast))
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(config.motorConfig().rotorToSensorRatio())
                    .withSensorToMechanismRatio(config.motorConfig().sensorToMechRatio()));

    motorConfigurator.apply(motorConfiguration);
  }

  @Override
  public void getUpdatedVals(PositionControllerValues values) {
    BaseStatusSignal.refreshAll(
        position, velocity, acceleration, motorVoltage, statorCurrent, supplyCurrent);

    values.position.mut_replace(position.getValueAsDouble(), Rotations).mut_plus(positionOffset);
    values.velocity.mut_replace(velocity.getValueAsDouble(), RotationsPerSecond);
    values.acceleration.mut_replace(acceleration.getValueAsDouble(), RotationsPerSecondPerSecond);
    values.motorVoltage.mut_replace(motorVoltage.getValueAsDouble(), Volts);
    values.statorCurrent.mut_replace(statorCurrent.getValueAsDouble(), Amps);
    values.supplyCurrent.mut_replace(supplyCurrent.getValueAsDouble(), Amps);
  }

  @Override
  public void setPosition(Angle newPos) {
    positionOffset = newPos.minus(position.getValue());
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
      motor.setControl(voltage.withOutput(setVoltage));
    } else {
      Angle motorPosition = position.getValue().plus(positionOffset);

      double feedbackVolts =
          feedback.calculate(motorPosition.in(Radians), setpointPosition.in(Radians));
      double feedforwardVolts =
          feedforward.calculate(
              setpointPosition.in(Radians), setpointVelocity.in(RadiansPerSecond));

      motor.setControl(voltage.withOutput(Volts.of(feedforwardVolts + feedbackVolts)));
    }
  }
}
