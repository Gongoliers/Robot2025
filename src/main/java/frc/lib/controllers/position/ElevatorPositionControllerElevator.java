package frc.lib.controllers.position;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.appliers.TalonFXConfigApplier;

/** Elevatr position controller used for elevator subsystem with 2 TalonFX motor controllers */
public class ElevatorPositionControllerElevator implements ElevatorPositionController {

  private final MechanismConfig config;

  private final TalonFX leader;
  private final TalonFX follower;

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<AngularAcceleration> acceleration;
  private final StatusSignal<Voltage> volts;
  private final StatusSignal<Current> amps;

  private final ElevatorFeedforward feedforward;
  private final PIDController feedback;

  private final VoltageOut voltage;

  private final double rotationsToMeters;
  private double posMeters = 0.0;
  private double posOffsetMeters = 0.0;

  private double setpointPosMeters = 0.0;
  private double setpointVelMetersPerSecond = 0.0;

  public ElevatorPositionControllerElevator(
      CAN leaderCAN,
      CAN followerCAN,
      MechanismConfig config,
      boolean enableFOC,
      boolean invertFollower,
      double rotationsToMeters) {
    
    this.config = config;

    // create hardware
    leader = new TalonFX(leaderCAN.id(), leaderCAN.bus());
    follower = new TalonFX(followerCAN.id(), followerCAN.bus());

    // status signals
    position = leader.getPosition();
    velocity = leader.getVelocity();
    acceleration = leader.getAcceleration();
    volts = leader.getMotorVoltage();
    amps = leader.getStatorCurrent();

    // create feedforward and feedback based on config
    feedforward = config.feedforwardControllerConfig().createElevatorFeedforward();
    feedback = config.feedbackControllerConfig().createPIDController();

    // set ratio of rotations of drum to meters of movement of elevator
    this.rotationsToMeters = rotationsToMeters;

    // default voltage
    voltage = new VoltageOut(0.0).withEnableFOC(enableFOC);
  }

  @Override
  public void configure() {
    BaseStatusSignal.setUpdateFrequencyForAll(100, position, velocity, acceleration);
    BaseStatusSignal.setUpdateFrequencyForAll(10, volts, amps);

    ParentDevice.optimizeBusUtilizationForAll(leader, follower);

    TalonFXConfigApplier.applyFactoryDefault(leader);
    TalonFXConfigApplier.apply(leader, config.motorConfig());

    TalonFXConfigApplier.applyFactoryDefault(follower);
    TalonFXConfigApplier.apply(follower, config.motorConfig());
  }

  @Override
  public void getUpdatedVals(ElevatorPositionControllerValues values) {
    BaseStatusSignal.refreshAll(position, velocity, acceleration, volts, amps);

    values.motorPosRotations = position.getValueAsDouble();
    values.motorVelRotationsPerSec = velocity.getValueAsDouble();
    values.motorAccRotationsPerSecPerSec = acceleration.getValueAsDouble();
    values.motorVolts = volts.getValueAsDouble();
    values.motorAmps = amps.getValueAsDouble();
    values.posMeters = posMeters;
    values.velMetersPerSec = velocity.getValueAsDouble()*rotationsToMeters;
    values.accMetersPerSecPerSec = acceleration.getValueAsDouble()*rotationsToMeters;
  }

  @Override
  public double getElevatorPos() {
    return posMeters;
  }

  @Override
  public void setElevatorPos(double posMeters) {
    posOffsetMeters = posMeters - this.posMeters;
  }

  @Override
  public void setSetpoint(double posMeters, double velMetersPerSec) {
    setpointPosMeters = posMeters;
    setpointVelMetersPerSecond = velMetersPerSec;
  }

  @Override
  public void periodic() {
    // update elevator position based on motor encoder position + offset
    posMeters = position.getValueAsDouble()*rotationsToMeters + posOffsetMeters;

    // approach setpoint
    double feedforwardVolts = feedforward.calculate(setpointVelMetersPerSecond);
    double feedbackVolts = feedback.calculate(position.getValueAsDouble()*rotationsToMeters, setpointPosMeters);

    leader.setControl(voltage.withOutput(feedforwardVolts + feedbackVolts));
  }
}
