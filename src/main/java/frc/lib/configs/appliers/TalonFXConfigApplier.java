package frc.lib.configs.appliers;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DriverStation;

import frc.lib.configs.MotorConfig;

/** Applies TalonFX configs */
public class TalonFXConfigApplier extends ConfigApplier {
  
  /**
   * Reports to the user that a TalonFX failed configuration
   * 
   * @param talonFX the TalonFX
   */
  private static void report(TalonFX talonFX) {
    DriverStation.reportWarning(
      "Failed to apply config to TalonFX with ID: " + talonFX.getDeviceID(), false);
  }

  /**
   * Applies a factory default config to a TalonFX
   * 
   * @param talonFX the TalonFX
   */
  public static void applyFactoryDefault(TalonFX talonFX) {
    TalonFXConfiguration factoryDefaults = new TalonFXConfiguration();

    TalonFXConfigurator configurator = talonFX.getConfigurator();

    if (attempt(() -> configurator.apply(factoryDefaults)) == false) {
      report(talonFX);
    }
  }

  /**
   * Creates the current limits configs for the motor config
   * 
   * @param motorConfig the motor config
   * @return the created current limits configs
   */
  private static CurrentLimitsConfigs createCurrentLimitsConfigs(MotorConfig motorConfig) {
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();

    currentLimitsConfigs.StatorCurrentLimit = motorConfig.statorCurrentLimit();
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    currentLimitsConfigs.SupplyCurrentLimit = motorConfig.supplyCurrentLimit();
    
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;

    return currentLimitsConfigs;
  }

  /**
   * Creates the feedback configs for the motor config
   * 
   * @param motorConfig the motor config
   * @return the created feedback configs
   */
  private static FeedbackConfigs createFeedbackConfigs(MotorConfig motorConfig) {
    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();

    // TODO This needs to be rethought if a fused/remote sensor is going to be used
    // By default, the sensor is the motor sensor, so we can use the motor sensor
    feedbackConfigs.SensorToMechanismRatio = motorConfig.motorToMechRatio();

    return feedbackConfigs;
  }

  /**
   * Creates the motor output configs for the motor config
   * 
   * @param motorConfig the motor config
   * @return the created motor output configs
   */
  private static MotorOutputConfigs createMotorOutputConfigs(MotorConfig motorConfig) {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();

    motorOutputConfigs.Inverted =
      motorConfig.ccwPositive()
        ? InvertedValue.CounterClockwise_Positive
        : InvertedValue.Clockwise_Positive;
    motorOutputConfigs.NeutralMode =
      motorConfig.neutralBrake() 
        ? NeutralModeValue.Brake 
        : NeutralModeValue.Coast;

    return motorOutputConfigs;
  }

  /**
   * Applies a motor config to a TalonFX
   * 
   * @param talonFX the TalonFX
   * @param motorConfig the motor config
   */
  public static void apply(TalonFX talonFX, MotorConfig motorConfig) {
    CurrentLimitsConfigs currentLimitsConfigs = createCurrentLimitsConfigs(motorConfig);
    FeedbackConfigs feedbackConfigs = createFeedbackConfigs(motorConfig);
    MotorOutputConfigs motorOutputConfigs = createMotorOutputConfigs(motorConfig);

    TalonFXConfigurator configurator = talonFX.getConfigurator();

    if (attempt(() -> configurator.apply(currentLimitsConfigs)) == false) {
      report(talonFX);
    }

    if (attempt(() -> configurator.apply(feedbackConfigs)) == false) {
      report(talonFX);
    }

    if (attempt(() -> configurator.apply(motorOutputConfigs)) == false) {
      report(talonFX);
    }
  }
}
