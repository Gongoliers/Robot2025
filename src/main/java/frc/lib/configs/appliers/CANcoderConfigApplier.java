package frc.lib.configs.appliers;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DriverStation;

import frc.lib.configs.AbsoluteEncoderConfig;

/** Applies CANcoder configs. */
public class CANcoderConfigApplier extends ConfigApplier {

  /**
   * Reports to the user that a CANcoder failed configuration.
   *
   * @param cancoder the CANcoder.
   */
  private static void report(CANcoder cancoder) {
    DriverStation.reportWarning(
        "Failed to apply config to CANcoder with ID: " + cancoder.getDeviceID(), false);
  }

  /**
   * Applies a factory default config to a CANcoder.
   *
   * @param cancoder the CANcoder.
   */
  public static void applyFactoryDefault(CANcoder cancoder) {
    CANcoderConfiguration factoryDefaults = new CANcoderConfiguration();

    CANcoderConfigurator configurator = cancoder.getConfigurator();

    if (attempt(() -> configurator.apply(factoryDefaults)) == false) {
      report(cancoder);
    }
  }

  /**
   * Creates the magnet sensor configs for the absolute encoder config.
   *
   * @param absoluteEncoderConfig the absolute encoder config.
   * @return the created magnet sensor configs.
   */
  private static MagnetSensorConfigs createMagnetSensorConfigs(
      AbsoluteEncoderConfig absoluteEncoderConfig) {
    final MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();

    magnetSensorConfigs.MagnetOffset = absoluteEncoderConfig.offset().getRotations();
    magnetSensorConfigs.SensorDirection =
        absoluteEncoderConfig.ccwPositive()
            ? SensorDirectionValue.CounterClockwise_Positive
            : SensorDirectionValue.Clockwise_Positive;

    return magnetSensorConfigs;
  }

  /**
   * Applies an absolute encoder config to a CANcoder.
   *
   * @param cancoder the CANcoder.
   * @param absoluteEncoderConfig the absolute encoder config.
   */
  public static void apply(CANcoder cancoder, AbsoluteEncoderConfig absoluteEncoderConfig) {
    MagnetSensorConfigs magnetSensorConfigs = createMagnetSensorConfigs(absoluteEncoderConfig);

    CANcoderConfigurator configurator = cancoder.getConfigurator();

    if (attempt(() -> configurator.apply(magnetSensorConfigs)) == false) {
      report(cancoder);
    }
  }
}