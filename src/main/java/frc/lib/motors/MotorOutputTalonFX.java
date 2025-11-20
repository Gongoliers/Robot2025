package frc.lib.motors;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.*;
import frc.lib.CAN;
import frc.lib.configs.MotorConfig;

/** Motor output implementation for a single TalonFX controlled motor */
public class MotorOutputTalonFX implements MotorOutput {

  /** Motor config */
  private final MotorConfig config;

  /** TalonFX hardware */
  private final TalonFX motor;

  /** Position offset */
  private final MutAngle positionOffset;

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
   * @param motorCAN CAN id and bus for TalonFX motor
   */
  public MotorOutputTalonFX(MotorConfig config, CAN motorCAN) {

    // set config
    this.config = config;

    // create hardware and status signals
    motor = new TalonFX(motorCAN.id(), motorCAN.bus());

    positionOffset = Rotations.mutable(0.0);

    position = motor.getPosition();
    velocity = motor.getVelocity();
    acceleration = motor.getAcceleration();
    motorVoltage = motor.getMotorVoltage();
    supplyVoltage = motor.getSupplyVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100,
        position,
        velocity,
        acceleration,
        motorVoltage,
        supplyVoltage,
        statorCurrent,
        supplyCurrent);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void setVoltage(Voltage voltage) {
    motor.setControl(this.voltage.withOutput(voltage));
  }

  @Override
  public void setPosition(Angle newPosition) {
    BaseStatusSignal.refreshAll(position);
    positionOffset.mut_replace(newPosition.minus(position.getValue()));
  }

  @Override
  public void updateValues(MotorValues values, Time dt) {
    BaseStatusSignal.refreshAll(
        position,
        velocity,
        acceleration,
        motorVoltage,
        supplyVoltage,
        statorCurrent,
        supplyCurrent);

    values.position.mut_replace(
        position.getValueAsDouble() + positionOffset.in(Rotations), Rotations);
    values.velocity.mut_replace(velocity.getValueAsDouble(), RotationsPerSecond);
    values.acceleration.mut_replace(acceleration.getValueAsDouble(), RotationsPerSecondPerSecond);
    values.motorVoltage.mut_replace(motorVoltage.getValueAsDouble(), Volts);
    values.supplyVoltage.mut_replace(supplyVoltage.getValueAsDouble(), Volts);
    values.statorCurrent.mut_replace(statorCurrent.getValueAsDouble(), Amps);
    values.supplyCurrent.mut_replace(supplyCurrent.getValueAsDouble(), Amps);
  }

  @Override
  public boolean configure() {
    // TODO Allow failed configurations to return false
    TalonFXConfigurator motorConfigurator = motor.getConfigurator();

    TalonFXConfiguration motorConfiguration =
        new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimit(config.statorCurrentLimit())
                    .withSupplyCurrentLimit(config.supplyCurrentLimit()))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(
                        config.ccwPositive()
                            ? InvertedValue.CounterClockwise_Positive
                            : InvertedValue.Clockwise_Positive)
                    .withNeutralMode(
                        config.neutralBrake() ? NeutralModeValue.Brake : NeutralModeValue.Coast))
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(config.rotorToSensorRatio())
                    .withSensorToMechanismRatio(config.sensorToMechRatio()));

    motorConfigurator.apply(motorConfiguration);

    return true;
  }
}
