package frc.lib.controllers.position;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
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

/** TalonFX position controller with a leader and follower motor (for elevator) */
public class PositionControllerTalonFX2Elevator implements PositionController {
  
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

  public PositionControllerTalonFX2Elevator(
      CAN leaderCAN,
      CAN followerCAN,
      MechanismConfig config,
      boolean enableFOC,
      boolean invertFollower) {
    
    this.config = config;

    leader = new TalonFX(leaderCAN.id(), leaderCAN.bus());
    follower = new TalonFX(followerCAN.id(), followerCAN.bus());
  }
}
