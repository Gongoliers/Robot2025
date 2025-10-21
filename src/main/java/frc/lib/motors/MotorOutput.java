package frc.lib.motors;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.ControlRequest;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.configs.MechanismConfig;

/** Simple interface that abstracts motor hardware (allowing us to sent a control request to a motor and get values back, no matter if it's one motor, two motors, or a simulated motor) */
public interface MotorOutput {
  
  //TODO: not sure if it's better to handle values like we did before, with a values class in the subsystem that is changed by the controller, or maybe try something new like this unchangin object of suppliers that gets returned directly from the controller
  /** Class that provides references to value suppliers from the real or simulated output motor(s) */
  public static class MotorValues {
    
    /** Gets current position */
    public Supplier<Angle> position;

    /** Gets current velocity */
    public Supplier<AngularVelocity> velocity;

    /** Gets current acceleration */
    public Supplier<AngularAcceleration> acceleration;

    /** Gets current armature voltage */
    public Supplier<Voltage> motorVoltage;

    /** Gets current supply voltage */
    public Supplier<Voltage> supplyVoltage;

    /** Gets current stator current */
    public Supplier<Current> statorCurrent;

    /** Gets current supply current */
    public Supplier<Current> supplyCurrent;

  }

  //TODO: there is a discussion to be had here, I like the idea of allowing usage of different control requests where applicable, but some different control requests might require some extra configuration, like slot gains for PositionVoltage
  /**
   * Sets the control request for motor output
   * 
   * @param controlRequest new control request
   */
  public void setControl(ControlRequest controlRequest);

  /**
   * Returns a values class with defined suppliers for different logged motor values
   * 
   * @return a values class with defined suppliers for different logged motor values
   */
  public MotorValues getValues();
  
  /**
   * Configures motor hardware
   * 
   * @param config config to configure motor hardware with
   * @return true if configuration was successful
   */
  public boolean configure(MechanismConfig config);
}
