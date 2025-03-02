package frc.lib.configs.appliers;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfigurator;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.DriverStation;

/** Applies CANrange configs */
public class CANrangeConfigApplier extends ConfigApplier {
  
  /**
   * Reports to the user that a CANrange failed configuration
   * 
   * @param canRange the CANrange that failed configuration
   */
  private static void report(CANrange canRange) {
    DriverStation.reportWarning(
      "Failed to apply config to Pigeon 2 with ID: " + canRange.getDeviceID(), false);
  }

  /**
   * Applies a factory default config to a CANrange
   * 
   * @param canRange
   */
  public static void applyFactoryDefault(CANrange canRange) {
    CANrangeConfiguration factoryDefaults = new CANrangeConfiguration();

    CANrangeConfigurator configurator = canRange.getConfigurator();

    if (attempt(() -> configurator.apply(factoryDefaults)) == false) {
      report(canRange);
    }
  }
}
