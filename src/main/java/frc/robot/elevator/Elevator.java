package frc.robot.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import frc.lib.Multithreaded;
import frc.lib.MultithreadedSubsystem;
import frc.lib.Subsystem;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotionProfileConfig.MotionProfileBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.controllers.position.PositionController;

public class Elevator extends MultithreadedSubsystem {

  /** Elevator subsystem singleton */
  private static Elevator instance = null;

  /** Elevator position controller */
  private final PositionController positionController;

  /** Ratio of distance units traveled by the elevator per angle unit of the position controller (defined as meters per rotation) */
  private final Per<DistanceUnit, AngleUnit> angleToDistance;

  /** Target elevator state */
  private ElevatorState targetState;

  /** Current elevator state */
  private ElevatorState currentState;

  /** Distance elevator should be from its target state before being considered in that state */
  private final Distance stateTolerance;

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

    angleToDistance = Meters.of(1).div(Rotations.of(1.0));

    currentState = ElevatorState.STOW;
    targetState = ElevatorState.STOW;

    stateTolerance = Meters.of(0.02);
  }

  @Override
  public void initializeTab() {
    
  }
  
  @Override
  public void periodic() {

  }

  @Override
  public void fastPeriodic() {

  }
}
