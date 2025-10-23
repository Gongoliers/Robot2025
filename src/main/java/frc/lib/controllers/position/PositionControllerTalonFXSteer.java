package frc.lib.controllers.position;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
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
import edu.wpi.first.wpilibj.DriverStation;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;

/** Position controller with TalonFX motor controller and CANcoder for absolute encoder used as a steer motor */
public class PositionControllerTalonFXSteer implements PositionController{

  /** Mechanism config */
  private final MechanismConfig config;
  
  /** TalonFX motor controller */
  private final TalonFX motor;

  /** CANcoder absolute encoder */
  private final CANcoder azimuth;

  /** Position of setpoint */
  private Angle setpointPosition;

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
  private final SimpleMotorFeedforward feedforward;

  /** Voltage control request */
  private final VoltageOut voltage;

  public PositionControllerTalonFXSteer(
      MechanismConfig config,
      CAN motorCAN,
      CAN encoderCAN) {
    
    // Set config
    this.config = config;

    // Initiialize hardware
    motor = new TalonFX(motorCAN.id(), motorCAN.bus());
    azimuth = new CANcoder(encoderCAN.id(), encoderCAN.bus());

    // Get status signals
    position = azimuth.getAbsolutePosition();
    velocity = azimuth.getVelocity();
    acceleration = motor.getAcceleration();
    motorVoltage = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity, acceleration, motorVoltage, statorCurrent, supplyCurrent);
    motor.optimizeBusUtilization();
    azimuth.optimizeBusUtilization();

    // Set up feedforward and feedback
    feedback = config.feedbackControllerConfig().createPIDController();
    feedforward = config.feedforwardControllerConfig().createSimpleMotorFeedforward();

    // Initialize other variable
    setpointPosition = Rotations.of(0.0);
    voltage = new VoltageOut(0.0);

    // Configure hardware
    configure();
  }

  public void configure() {
    TalonFXConfigurator motorConfigurator = motor.getConfigurator();
    CANcoderConfigurator encoderConfigurator = azimuth.getConfigurator();
    
    TalonFXConfiguration motorConfiguration = new TalonFXConfiguration()
      .withCurrentLimits(new CurrentLimitsConfigs()
        .withStatorCurrentLimit(config.motorConfig().statorCurrentLimit())
        .withSupplyCurrentLimit(config.motorConfig().supplyCurrentLimit()))
      .withMotorOutput(new MotorOutputConfigs()
        .withInverted(config.motorConfig().ccwPositive() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive)
        .withNeutralMode(config.motorConfig().neutralBrake() ? NeutralModeValue.Brake : NeutralModeValue.Coast))
      .withFeedback(new FeedbackConfigs()
        .withRotorToSensorRatio(config.motorConfig().rotorToSensorRatio())
        .withSensorToMechanismRatio(config.motorConfig().sensorToMechRatio()));

    CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration()
      .withMagnetSensor(new MagnetSensorConfigs()
        .withMagnetOffset(config.absoluteEncoderConfig().sensorToMechRatio()));

    motorConfigurator.apply(motorConfiguration);
    encoderConfigurator.apply(encoderConfiguration);
  }

  public void getUpdatedVals(PositionControllerValues values) {
    BaseStatusSignal.refreshAll(position, velocity, acceleration, motorVoltage, statorCurrent, supplyCurrent);
    
    values.position.mut_replace(position.getValue());
    values.velocity.mut_replace(velocity.getValue());
    values.acceleration.mut_replace(acceleration.getValue());
    values.motorVoltage.mut_replace(motorVoltage.getValue());
    values.statorCurrent.mut_replace(statorCurrent.getValue());
    values.supplyCurrent.mut_replace(supplyCurrent.getValue());
  }

  public void setPosition(Angle newPos) {
    azimuth.setPosition(newPos);
  }

  public void setSetpoint(Angle setpointPosition, AngularVelocity setpointVelocity) {
    this.setpointPosition = setpointPosition;
  }

  public void setVoltage(Voltage volts) {
    DriverStation.reportWarning("There should be no reason to set or reset the voltage of a steer motor directly", true);
  }

  public void clearVoltage() {
    DriverStation.reportWarning("There should be no reason to set or reset the voltage of a steer motor directly", true);
  }

  private double calculateFeedforward(double measurement, double setpoint) {
    return (feedback.atSetpoint() ? ((measurement > setpoint) ? feedforward.getKs() : -feedforward.getKs()) : 0.0);
  }

  public void periodic() {
    Angle motorPosition = position.getValue();

    double feedbackVolts = feedback.calculate(motorPosition.in(Radians), setpointPosition.in(Radians));
    double feedforwardVolts = calculateFeedforward(motorPosition.in(Radians), setpointPosition.in(Radians));

    motor.setControl(voltage.withOutput(feedforwardVolts + feedbackVolts));
  }
}
