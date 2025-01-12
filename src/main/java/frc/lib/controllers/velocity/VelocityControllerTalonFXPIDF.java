package frc.lib.controllers.velocity;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
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
import frc.lib.configs.appliers.TalonFXConfigApplier;

/** TalonFX velocity controller that makes use of PID feedback and feedforward */
public class VelocityControllerTalonFXPIDF implements VelocityController {
  
  private final MechanismConfig config;

  private final TalonFX motor;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> volts;
  private final StatusSignal<Current> amps;

  private final SimpleMotorFeedforward feedforward;

  private final PIDController feedback;
  
  private final VoltageOut voltage;

  public VelocityControllerTalonFXPIDF(
      CAN motorCAN,
      MechanismConfig config,
      boolean enableFOC) {
    
    this.config = config;
    
    // create hardware
    motor = new TalonFX(motorCAN.id(), motorCAN.bus());
    
    // status signals (values in memory updated at a fixed rate by hardware over CAN)
    position = motor.getPosition();
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

    ParentDevice.optimizeBusUtilizationForAll(motor);

    TalonFXConfigApplier.applyFactoryDefault(motor);
    TalonFXConfigApplier.apply(motor, config.motorConfig());
  }

  @Override
  public void getUpdatedVals(VelocityControllerValues values) {
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
  public void setSetpoint(double velRotationsPerSec) {
    double feedforwardVolts = feedforward.calculate(velRotationsPerSec);

    double measuredVelRotationsPerSec = velocity.getValueAsDouble();

    double feedbackVolts = feedback.calculate(measuredVelRotationsPerSec, velRotationsPerSec);

    motor.setControl(voltage.withOutput(feedforwardVolts + feedbackVolts));
  }

  @Override
  public void periodic() {}
}
