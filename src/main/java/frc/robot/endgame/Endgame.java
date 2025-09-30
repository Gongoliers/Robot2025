package frc.robot.endgame;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.Subsystem;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotionProfileConfig.MotionProfileBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.controllers.position.EndgamePositionController;
import frc.lib.controllers.position.PositionController;
import frc.lib.controllers.position.EndgamePositionController.EndgamePositionControllerValues;
import frc.lib.controllers.position.PositionController.PositionControllerValues;
import frc.lib.sendables.EndgameStateSendable;
import frc.lib.sendables.PivotStateSendable;
import frc.robot.RobotConstants;

public class Endgame extends Subsystem {
  
  /** Endgame motor */
  private final EndgamePositionController motor;

  /** Motor values */
  private EndgamePositionControllerValues motorValues = new EndgamePositionControllerValues();

  /** Endgame motion profile */
  private final TrapezoidProfile endgameProfile;

  /** Target endgame state */
  private EndgameState targetState;

  /** Current endgame state */
  private EndgameState currentState;

  /** Endgame state tolerance (max distance to target state to be considered at that state) */
  private double stateTolerance;

  /** Profile endgame setpoint */
  private TrapezoidProfile.State profiledSetpoint;

  /** Ideal endgame position (follows motion profile perfectly) */
  private double idealPosRotations;

  /** Ideal endgame velocity (follows motion profile perfectly) */
  private double idealVelRotationsPerSec;

  /** Engame config */
  private final MechanismConfig endgameConfig =
    MechanismBuilder.defaults()
      .motorConfig(
        MotorBuilder.defaults()
          .ccwPositive(false)
          .motorToMechRatio(500)
          .statorCurrentLimit(200)
          .neutralBrake(true)
          .build())
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kA(0)
          .kG(0)
          .kS(0.167)
          .kV(9.2)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(2)
          .kI(0)
          .kD(0)
          .build())
      .motionProfileConfig(
        MotionProfileBuilder.defaults()
          .maxVelocity(0.175)
          .maxAcceleration(1)
          .build())
      .build();

  /** Initializes endgame subsystem */
  public Endgame() {
    motor = EndgameFactory.createEndgameMotor(endgameConfig);
    motor.configure();

    endgameProfile = endgameConfig.motionProfileConfig().createTrapezoidProfile();

    targetState = EndgameState.STOW;
    currentState = EndgameState.STOW;
    stateTolerance = 0.03;

    profiledSetpoint = new TrapezoidProfile.State();
    idealPosRotations = -0.25;
    idealVelRotationsPerSec = 0.0;

    motor.clearSetVoltage();
    motor.setPos(-0.25);
  }

  @Override
  public void initializeTab() {
    // Get tab
    ShuffleboardTab tab = Shuffleboard.getTab("Endgame");

    // State info
    tab.add("Target state", new EndgameStateSendable(() -> targetState));
    tab.add("Current state", new EndgameStateSendable(() -> currentState));
    tab.addBoolean("At target state", () -> targetState == currentState);

    // Get current values column
    ShuffleboardLayout values = tab.getLayout("Current values", BuiltInLayouts.kList);

    values.addDouble("Pos (rot)", () -> motorValues.posRotations);
    values.addDouble("Vel (rotps)", () -> motorValues.velRotationsPerSec);
    values.addDouble("Acc (rotpsps)", () -> motorValues.accRotationsPerSecPerSec);
    values.addDouble("Volts", () -> motorValues.motorVolts);
    values.addDouble("Amps", () -> motorValues.motorAmps);

    // Get ideal values column
    ShuffleboardLayout idealValues = tab.getLayout("Ideal values", BuiltInLayouts.kList);

    idealValues.addDouble("Pos (rot)", () -> idealPosRotations);
    idealValues.addDouble("vel (rotps)", () -> idealVelRotationsPerSec);
  }

  @Override
  public void periodic() {
    // Approach setpoint
    motor.getUpdatedVals(motorValues);   
    
    profiledSetpoint = calculateSetpoint();
    idealPosRotations = profiledSetpoint.position;
    idealVelRotationsPerSec = profiledSetpoint.velocity;

    motor.setSetpoint(profiledSetpoint.position, profiledSetpoint.velocity);

    // Update current pivot state if within tolerance
    if (Math.abs(motorValues.posRotations - targetState.getPosRotations()) <= stateTolerance) {
      currentState = targetState;
    } else {
      currentState = EndgameState.MOVING;
    }

    if (motorValues.posRotations >= 0.55) {
      motor.clearSetVoltage();
    }

    motor.periodic();
  }

  private TrapezoidProfile.State calculateSetpoint() {
    return endgameProfile.calculate(
      RobotConstants.PERIODIC_DURATION, 
      new TrapezoidProfile.State(idealPosRotations, idealVelRotationsPerSec), 
      new TrapezoidProfile.State(targetState.getPosRotations(), 0.0));
  }

  /** Get current endgame pos in rotations */
  public double getPosRotations() {
    return motorValues.posRotations;
  }

  /** Get current endgame state */
  public EndgameState getState() {
    return currentState;
  }

  /** Gets target endgame state */
  public EndgameState getTargetState() {
    return targetState;
  }

  /** Sets target endgame state */
  public void setTargetState(EndgameState state) {
    targetState = state;
  }

  /** Returns true if at target endgame state */
  public boolean atTargetState() {
    return targetState == currentState;
  }

  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public Command zero() {
    return Commands.runOnce(
      () -> {
        motor.setPos(0.0);
      });
  }
}
