package frc.robot.pivot;

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
import frc.lib.controllers.position.PositionController;
import frc.lib.controllers.position.PositionController.PositionControllerValues;
import frc.lib.sendables.PivotStateSendable;
import frc.robot.RobotConstants;

public class Pivot extends Subsystem{
  
  /** Pivot subsystem instance */
  private static Pivot instance = null;

  /** Pivot motor */
  private final PositionController motor;

  /** Motor values */
  private PositionControllerValues motorValues = new PositionControllerValues();

  /** Pivot motion profile */
  private final TrapezoidProfile pivotProfile;

  /** Target pivot state */
  private PivotState targetState;

  /** Current pivot state */
  private PivotState currentState;

  /** Pivot state tolerance (max distance to target state to be considered at that state) */
  private double stateTolerance;

  /** Profile pivot setpoint */
  private TrapezoidProfile.State profiledSetpoint;

  /** Ideal pivot position (follows motion profile perfectly) */
  private double idealPosRotations;

  /** Ideal pivot velocity (follows motion profile perfectly) */
  private double idealVelRotationsPerSec;

  /** Pivot config */
  private final MechanismConfig pivotConfig =
    MechanismBuilder.defaults()
      .motorConfig(
        MotorBuilder.defaults()
          .ccwPositive(true)
          .motorToMechRatio((58/10)*(58/18)*(30/12))
          .statorCurrentLimit(50)
          .build())
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kA(1)
          .kG(0.2145)
          .kS(0.0285)
          .kV(0.638)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(15)
          .kI(0.0)
          .kD(0.0)
          .build())
      .motionProfileConfig(
        MotionProfileBuilder.defaults()
          .maxVelocity(1)
          .maxAcceleration(2)
          .build())
      .build();

  /** Gets pivot subsystem instance */
  public static Pivot getInstance() {
    if (instance == null) {
      instance = new Pivot();
    }

    return instance;
  }

  /** Initializes pivot subsystem */
  private Pivot() {
    motor = PivotFactory.createPivotMotor(pivotConfig);
    motor.configure();

    pivotProfile = pivotConfig.motionProfileConfig().createTrapezoidProfile();

    targetState = PivotState.STOW;
    currentState = PivotState.STOW;
    stateTolerance = 0.01;

    profiledSetpoint = new TrapezoidProfile.State();
    idealPosRotations = 0.0;
    idealVelRotationsPerSec = 0.0;

    motor.setPos(0.0);
  }

  @Override
  public void initializeTab() {
    // Get tab
    ShuffleboardTab tab = Shuffleboard.getTab("Pivot");

    // State info
    tab.add("Target state", new PivotStateSendable(() -> targetState));
    tab.add("Current state", new PivotStateSendable(() -> currentState));
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
      currentState = PivotState.MOVING;
    }

    motor.periodic();
  }

  private TrapezoidProfile.State calculateSetpoint() {
    return pivotProfile.calculate(
      RobotConstants.PERIODIC_DURATION, 
      new TrapezoidProfile.State(idealPosRotations, idealVelRotationsPerSec), 
      new TrapezoidProfile.State(targetState.getPosRotations(), 0.0));
  }

  /** Get current pivot pos in rotations */
  public double getPosRotations() {
    return motorValues.posRotations;
  }

  /** Get current pivot state */
  public PivotState getState() {
    return currentState;
  }

  /** Gets target pivot state */
  public PivotState getTargetState() {
    return targetState;
  }

  /** Sets target pivot state */
  public void setTargetState(PivotState state) {
    targetState = state;
  }

  /** Returns true if at target pivot state */
  public boolean atTargetState() {
    return targetState == currentState;
  }

  public Command zero() {
    return Commands.runOnce(
      () -> {
        motor.setPos(0.0);
      });
  }
}