package frc.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.MultithreadedSubsystem;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotionProfileConfig.MotionProfileBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.controllers.position.PositionController;
import frc.lib.controllers.position.PositionController.PositionControllerValues;
import frc.robot.RobotConstants;

public class Elevator extends MultithreadedSubsystem {

  /** Elevator subsystem singleton */
  private static Elevator instance = null;

  /** Elevator position controller */
  private final PositionController positionController;

  /** Elevator position controller values */
  private PositionControllerValues positionControllerValues;

  /** Ratio of meters travelled by elevator per rotations made by position controller */
  private final double rotationsToMeters;

  /** Target elevator state */
  private ElevatorState targetState;

  /** Current elevator state */
  private ElevatorState currentState;

  /** Distance elevator should be from its target state before being considered in that state */
  private final Distance stateTolerance;

  /** Trapezoid motion profile */
  private final TrapezoidProfile motionProfile;

  /** Setpoint that follows trapezoid motion profile */
  private TrapezoidProfile.State profiledSetpoint;

  /** Mechanism config */
  private final MechanismConfig config = MechanismBuilder.defaults()
    .feedforwardControllerConfig(FeedforwardControllerBuilder.defaults()
      .kV(0.0)
      .kA(0.0)
      .kG(0.0)
      .kS(0.0)
      .build())
    .feedbackControllerConfig(FeedbackControllerBuilder.defaults()
      .kP(0.0)
      .kI(0.0)
      .kD(0.0)
      .build())
    .motionProfileConfig(MotionProfileBuilder.defaults()
      .maxVelocity(2)
      .maxAcceleration(2)
      .build())
    .motorConfig(MotorBuilder.defaults()
      .ccwPositive(false)
      .rotorToSensorRatio(1.0)
      .sensorToMechRatio(1.0)
      .neutralBrake(true)
      .statorCurrentLimit(80.0)
      .supplyCurrentLimit(40.0)
      .build())
    .build();

  /**
   * Gets reference to instance of elevator subsystem singleton
   * 
   * @return reference to instance of elevator subystem singleton
   */
  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }

    return instance;
  }

  /** Elevator subsystem constructor */
  private Elevator() {
    positionController = ElevatorFactory.createElevatorPositionController(config);

    rotationsToMeters = 0.02/1;

    currentState = ElevatorState.STOW;
    targetState = ElevatorState.STOW;

    stateTolerance = Meters.of(0.02);

    motionProfile = config.motionProfileConfig().createTrapezoidProfile();
    profiledSetpoint = new TrapezoidProfile.State(0, 0);
  }

  @Override
  public void initializeTab() {
    
  }
  
  @Override
  public void periodic() {

  }

  @Override
  public void fastPeriodic() {
    positionController.getUpdatedVals(positionControllerValues);

    Distance position = Meters.of(positionControllerValues.position.in(Rotations) * rotationsToMeters);

    if (MathUtil.isNear(targetState.getPosMeters(), position.in(Meters), stateTolerance.in(Meters))) {
      // If close enough to target state, consider the eleevator to be at that state
      currentState = targetState;
    } else {
      // If not, conisder hteelevator to be moving
      currentState = ElevatorState.MOVING;
    }

    if (targetState == ElevatorState.STOW && MathUtil.isNear(0.0, position.in(Meters), 0.04)) {
      // If near enough to stow position and you want to stow, disable the motors to prevent stalling
      positionController.setVoltage(Volts.of(0.01));
    } else {
      positionController.clearVoltage();

      if (currentState != targetState) {
        // If not at target state yet, approach state with motion profile
        profiledSetpoint = motionProfile.calculate(
            RobotConstants.FAST_PERIODIC_DURATION, 
            profiledSetpoint, 
            new TrapezoidProfile.State(targetState.getPosMeters(), 0));

        positionController.setSetpoint(
            Rotations.of(profiledSetpoint.position / rotationsToMeters), 
            RotationsPerSecond.of(profiledSetpoint.velocity / rotationsToMeters));
      } else {
        // If reached target state, set setpont to hold at that state
        positionController.setSetpoint(
            Rotations.of(targetState.getPosMeters() / rotationsToMeters), 
            RotationsPerSecond.of(0));
      }
    }
  }

  /**
   * Returns true if elevator is at its target state
   * 
   * @return true if elevator is at its target state
   */
  public boolean atTargetState() {
    return currentState == targetState;
  }

  /**
   * Returns a command that sets the target state of the elevator
   * 
   * @param newTargetState new target state
   * @return a command that sets the target state of the elevator
   */
  public Command setTargetState(ElevatorState newTargetState) {
    return Commands.runOnce(() -> {
      targetState = newTargetState;
    }, this);
  }
  
  /**
   * Returns a command that sets the target state of the elevator and waits until it reaches that state
   * 
   * @param targetState target elevator state
   * @return a command that sets the target state of the elevator and waits until it reaches that state
   */
  public Command goToState(ElevatorState targetState) {
    return setTargetState(targetState).andThen(Commands.waitUntil(this::atTargetState));
  }
}
