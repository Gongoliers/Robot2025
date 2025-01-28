package frc.lib.controllers.position;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.appliers.CANcoderConfigApplier;
import frc.lib.configs.appliers.TalonFXConfigApplier;

/** TalonFX position controller used for steering on a swerve module */
public class PositionControllerTalonFXSteer implements PositionController {
  
  private final MechanismConfig config;

  private final TalonFX motor;
  
  private final CANcoder azimuthEncoder;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> volts;
  private final StatusSignal<Current> amps;

  private final SimpleMotorFeedforward feedforward;

  private final PIDController feedback;

  private final VoltageOut voltage;

  private double setpointPosRotations;
  private double setpointVelRotationsPerSec;

  public PositionControllerTalonFXSteer(
      CAN steerCAN,
      CAN encoderCAN,
      MechanismConfig config,
      boolean enableFOC) {
    
    this.config = config;

    // create hardware
    motor = new TalonFX(steerCAN.id(), steerCAN.bus());
    azimuthEncoder = new CANcoder(encoderCAN.id(), encoderCAN.bus());

    // status signals
    position = azimuthEncoder.getAbsolutePosition();
    velocity = motor.getVelocity();
    acceleration = motor.getAcceleration();
    volts = motor.getMotorVoltage();
    amps = motor.getStatorCurrent();

    // create feedforward and feedback based on config
    feedforward = config.feedforwardControllerConfig().createSimpleMotorFeedforward();
    feedback = config.feedbackControllerConfig().createPIDController();

    // default voltage
    voltage = new VoltageOut(0.0).withEnableFOC(enableFOC);
  }

  @Override
  public void configure() {
    BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity, acceleration);
    BaseStatusSignal.setUpdateFrequencyForAll(10, volts, amps);

    ParentDevice.optimizeBusUtilizationForAll(motor, azimuthEncoder);

    TalonFXConfigApplier.applyFactoryDefault(motor);
    TalonFXConfigApplier.apply(motor, config.motorConfig());

    CANcoderConfigApplier.applyFactoryDefault(azimuthEncoder);
    CANcoderConfigApplier.apply(azimuthEncoder, config.absoluteEncoderConfig());
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
  public void setPos(double posRotations) {}

  @Override
  public void setSetpoint(double posRotations, double velRotationsPerSec) {
    setpointPosRotations = posRotations;
    setpointVelRotationsPerSec = velRotationsPerSec;
  }

  @Override
  public void periodic() {
    double measuredPosRotations = position.getValueAsDouble();
    double feedforwardVolts = calculateFeedforward(measuredPosRotations, setpointPosRotations);
    double feedbackVolts = feedback.calculate(measuredPosRotations, setpointPosRotations);

    motor.setControl(voltage.withOutput(feedforwardVolts + feedbackVolts));
  }

  private double calculateFeedforward(double measurementRotations, double setpointRotations) {
    if (feedback.atSetpoint() == false) {
      if (measurementRotations > setpointRotations) {
        return feedforward.getKs();
      } else {
        return -feedforward.getKs();
      }
    }

    return 0.0;
  }
}
