package frc.lib.controllers.position;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MeasurementHealthValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.appliers.TalonFXConfigApplier;

/** TalonFX position controller for aiming the manipulator */
public class PositionControllerTalonFXPivot implements PositionController {
  
  private final MechanismConfig config;

  private final TalonFX motor;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> volts;
  private final StatusSignal<Current> amps;

  private final ArmFeedforward feedforward;

  private final PIDController feedback;

  private final VoltageOut voltage;

  private double setpointPosRotations = 0.0;
  private double setpointVelRotationsPerSec = 0.0;

  public PositionControllerTalonFXPivot(
      CAN motorCAN,
      MechanismConfig config,
      boolean ccwPositive,
      boolean enableFOC) {

    this.config = config;

    // create hardware
    motor = new TalonFX(motorCAN.id(), motorCAN.bus());

    // status signals
    position = motor.getPosition();
    velocity = motor.getVelocity();
    acceleration = motor.getAcceleration();
    volts = motor.getMotorVoltage();
    amps = motor.getStatorCurrent();

    // create feedforward and feedback based on config
    feedforward = config.feedforwardControllerConfig().createArmFeedforward();
    feedback = config.feedbackControllerConfig().createPIDController();
    feedback.setTolerance(0.005);

    // default voltage
    voltage = new VoltageOut(0.0).withEnableFOC(enableFOC);
  }

  @Override
  public void configure() {
    BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity, acceleration);
    BaseStatusSignal.setUpdateFrequencyForAll(50, volts, amps);

    TalonFXConfigApplier.applyFactoryDefault(motor);
    TalonFXConfigApplier.apply(motor, config.motorConfig());
  }

  @Override
  public void getUpdatedVals(PositionControllerValues values) {
    BaseStatusSignal.refreshAll(position, velocity, acceleration, volts, amps);

    values.posRotations = position.getValueAsDouble();
    values.velRotationsPerSec = velocity.getValueAsDouble();
    values.accRotationsPerSecPerSec = acceleration.getValueAsDouble();
    values.motorVolts = volts.getValueAsDouble();
    values.motorAmps = amps.getValueAsDouble();
  }

  @Override
  public void setPos(double posRotations) {
    motor.setPosition(posRotations);
  }

  @Override
  public void setSetpoint(double posRotations, double velRotationsPerSec) {
    setpointPosRotations = posRotations;
    setpointVelRotationsPerSec = velRotationsPerSec;
  }

  @Override
  public void periodic() {
    // approach setpoint
    double measuredPosRotations = position.getValueAsDouble();
    double feedforwardVolts = feedforward.calculate(setpointPosRotations * Math.PI * 2, -setpointVelRotationsPerSec * Math.PI * 2);
    double feedbackVolts = feedback.calculate(measuredPosRotations, setpointPosRotations);

    motor.setControl(voltage.withOutput(-feedforwardVolts + feedbackVolts));
  }
}
