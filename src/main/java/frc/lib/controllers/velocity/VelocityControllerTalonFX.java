package frc.lib.controllers.velocity;

import static edu.wpi.first.units.Units.RadiansPerSecond;

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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;

/** Velocity controller implementation for roller driven by one TalonFX */
public class VelocityControllerTalonFX implements VelocityController {

  /** Mechanism config */
  private final MechanismConfig config;

  /** TalonFX controller motor */
  private final TalonFX motor;

  /** Setpoint velocity */
  private AngularVelocity setpointVelocity;

  // Status signals
  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Current> supplyCurrent;

  /** PID controller for feedback */
  private final PIDController feedback;

  /** Feedforward controller */
  private final SimpleMotorFeedforward feedforward;

  /** Voltage request object */
  private final VoltageOut voltage;

  /** True if a voltage is manually set */
  private boolean voltageSet;

  /** Value of manually set voltage */
  private Voltage setVoltage;

  public VelocityControllerTalonFX(MechanismConfig config, CAN motorCAN) {

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
    feedforward = config.feedforwardControllerConfig().createSimpleMotorFeedforward();

    // Other initialization
    voltage = new VoltageOut(0.0);

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
  public void getUpdatedVals(VelocityControllerValues values) {
    BaseStatusSignal.refreshAll(
        position, velocity, acceleration, motorVoltage, statorCurrent, supplyCurrent);

    values.position.mut_replace(position.getValue());
    values.velocity.mut_replace(velocity.getValue());
    values.acceleration.mut_replace(acceleration.getValue());
    values.motorVoltage.mut_replace(motorVoltage.getValue());
    values.statorCurrent.mut_replace(statorCurrent.getValue());
    values.supplyCurrent.mut_replace(supplyCurrent.getValue());
  }

  @Override
  public void setSetpoint(AngularVelocity velocity) {
    setpointVelocity = velocity;
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
      AngularVelocity motorVelocity = velocity.getValue();

      double feedbackVolts =
          feedback.calculate(
              motorVelocity.in(RadiansPerSecond), setpointVelocity.in(RadiansPerSecond));
      double feedforwardVolts = feedforward.calculate(setpointVelocity.in(RadiansPerSecond));

      motor.setControl(voltage.withOutput(feedforwardVolts + feedbackVolts));
    }
  }
}
