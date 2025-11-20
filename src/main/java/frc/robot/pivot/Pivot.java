package frc.robot.pivot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.MultithreadedSubsystem;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotionProfileConfig.MotionProfileBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.controllers.position.PositionController;
import frc.lib.controllers.position.PositionController.PositionControllerValues;
import frc.robot.RobotConstants;

public class Pivot extends MultithreadedSubsystem {

  /** Pivot subsystem singleton */
  private static Pivot instance = null;

  /** Pivot position controller */
  private final PositionController positionController;

  /** Pivot position controller values */
  private PositionControllerValues positionControllerValues = new PositionControllerValues();

  /** Lock object used to maintain thread safety */
  private final Object lock = new Object();

  /** Target pivot state */
  private PivotState targetState;

  /** Current pivot state */
  private PivotState currentState;

  /** Ditsance pivot should be from its target state before being considered at that state */
  private final Angle stateTolerance;

  /** Trapezoid motion profile */
  private final TrapezoidProfile motionProfile;

  /** Setpoint that follows trapezoid motion profile */
  private TrapezoidProfile.State profiledSetpoint;

  /** Mechanism config */
  private final MechanismConfig config =
      MechanismBuilder.defaults()
          .feedforwardControllerConfig(
              FeedforwardControllerBuilder.defaults().kV(0.0).kA(0.0).kG(0.0).kS(0.0).build())
          .feedbackControllerConfig(
              FeedbackControllerBuilder.defaults().kP(0.0).kI(0.0).kD(0.0).build())
          .motionProfileConfig(
              MotionProfileBuilder.defaults().maxVelocity(2).maxAcceleration(2).build())
          .motorConfig(
              MotorBuilder.defaults()
                  .ccwPositive(false)
                  .rotorToSensorRatio(1.0)
                  .sensorToMechRatio(1.0)
                  .neutralBrake(true)
                  .statorCurrentLimit(80.0)
                  .supplyCurrentLimit(40.0)
                  .build())
          .build();

  /**
   * Gets instance of pivot subsystem singleton
   *
   * @return instance of pivot subsystem singleton
   */
  public static Pivot getInstance() {
    if (instance == null) {
      instance = new Pivot();
    }

    return instance;
  }

  /** Pivot subsystem constructor */
  private Pivot() {
    positionController = PivotFactory.createPivotPositionController(config);

    currentState = PivotState.STOW;
    targetState = PivotState.STOW;

    stateTolerance = Rotations.of(0.01);

    motionProfile = config.motionProfileConfig().createTrapezoidProfile();
    profiledSetpoint = new TrapezoidProfile.State(targetState.getPosRotations(), 0);
  }

  @Override
  public void initializeTab() {
    // Get tab
    ShuffleboardTab tab = Shuffleboard.getTab("Pivot");

    // State info
    tab.addString("Target state", () -> targetState.name());
    tab.addString("Current state", () -> currentState.name());
    tab.addBoolean("At target state", () -> targetState == currentState);

    // Setpoint column
    ShuffleboardLayout setpointColumn = tab.getLayout("Setpoint", BuiltInLayouts.kList);

    setpointColumn.addDouble("Setpoint position (rot)", () -> profiledSetpoint.position);
    setpointColumn.addDouble("Setpoint velocity (rot/s)", () -> profiledSetpoint.velocity);

    // Current state column
    ShuffleboardLayout stateColumn = tab.getLayout("Current state", BuiltInLayouts.kList);

    stateColumn.addDouble(
        "Pivot position (rot)", () -> positionControllerValues.position.in(Rotations));
    stateColumn.addDouble(
        "Pivot velocity (rot/s)", () -> positionControllerValues.velocity.in(RotationsPerSecond));
    stateColumn.addDouble(
        "Pivot acceleration (rot/s/s)",
        () -> positionControllerValues.acceleration.in(RotationsPerSecondPerSecond));
    stateColumn.addDouble("Motor voltage", () -> positionControllerValues.motorVoltage.in(Volts));
    stateColumn.addDouble("Stator current", () -> positionControllerValues.statorCurrent.in(Amps));
    stateColumn.addDouble("Supply current", () -> positionControllerValues.supplyCurrent.in(Amps));
  }

  @Override
  public void periodic() {

    // Get the current position of the controller
    Angle position;
    synchronized (lock) {
      position = positionControllerValues.position;
    }

    if (MathUtil.isNear(
        targetState.getPosRotations(), position.in(Rotations), stateTolerance.in(Rotations))) {
      // If close enough to target state, consider the pivot to be at that state
      currentState = targetState;
    } else {
      // If not, consider the pivot to be moving
      currentState = PivotState.MOVING;
    }

    if (currentState != targetState) {
      // If not at target state yet, approach state with motion profile
      profiledSetpoint =
          motionProfile.calculate(
              RobotConstants.PERIODIC_DURATION,
              profiledSetpoint,
              new TrapezoidProfile.State(targetState.getPosRotations(), 0));

      positionController.setSetpoint(
          Rotations.of(profiledSetpoint.position),
          RotationsPerSecond.of(profiledSetpoint.velocity));
    } else {
      // If reached target state, set setpoint to hold at that state
      positionController.setSetpoint(
          Rotations.of(targetState.getPosRotations()), RotationsPerSecond.of(0.0));
    }

    positionController.periodic();
  }

  @Override
  public void fastPeriodic() {
    synchronized (lock) {
      positionController.getUpdatedVals(positionControllerValues);
    }
  }
}
