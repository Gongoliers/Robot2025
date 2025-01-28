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

/** Linear position controller used for elevator subsystem with 2 TalonFX motor controllers */
public class LinearPositionControllerElevator implements LinearPositionController {

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

  private double elevatorPos = 0.0;

  public LinearPositionControllerElevator(
      CAN leaderCAN,
      CAN followerCAN,
      MechanismConfig config,
      boolean enableFOC,
      boolean invertFollower) {
    
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
  public void getUpdatedVals(LinearPositionControllerValues values) {
    BaseStatusSignal.refreshAll(position, velocity, acceleration, volts, amps);

    values.motorPosRotations = position.getValueAsDouble();
    values.motorVelRotationsPerSec = velocity.getValueAsDouble();
    values.motorAccRotationsPerSecPerSec = acceleration.getValueAsDouble();
    values.motorVolts = volts.getValueAsDouble();
    values.motorAmps = amps.getValueAsDouble();
    values.posMeters = elevatorPos;
  }

  @Override
  public void setPos(double posMeters) {
    elevatorPos = posMeters;
  }

  @Override
  public void setSetpoint(double posMeters, double velMetersPerSec) {
    double feedforwardVolts = calculateFeedforward(elevatorPos, posMeters);
    
    double feedbackVolts = feedback.calculate(elevatorPos, posMeters);

    leader.setControl(voltage.withOutput(feedforwardVolts + feedbackVolts));
  }

  @Override
  public void periodic() {
    // update elevator position based on motor velocity
    setPos(elevatorPos + (velocity.getValueAsDouble() / config.motorConfig().motorToMechRatio()));
  }

  private double calculateFeedforward(double measurementMeters, double setpointMeters) {
    if (feedback.atSetpoint() == false) {
      if (measurementMeters > setpointMeters) {
        return feedforward.getKs() + feedforward.getKg();
      } else {
        return -feedforward.getKs() + feedforward.getKg();
      }
    }

    return 0.0;
  }
}
