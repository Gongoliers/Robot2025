package frc.robot.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotionProfileConfig.MotionProfileBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.controllers.position.ElevatorPositionController;
import frc.lib.controllers.position.ElevatorPositionController.ElevatorPositionControllerValues;
import frc.lib.sendables.ElevatorStateSendable;
import frc.robot.RobotConstants;

/** Elevator subsystem */
public class Elevator extends Subsystem {

  /** Elevator singleton */
  private static Elevator instance = null;

  /** Motor(s) that drives the elevator */
  private final ElevatorPositionController motor;

  /** Current elevator state */
  private ElevatorState targetState = ElevatorState.STOW;
  private ElevatorState currentState = ElevatorState.STOW;

  /** Profiled elevator setpoint */
  private State profiledSetpoint = new State();

  /** Trapezoidal motion profile for smooth movement from setpoint to setpoint */
  private final TrapezoidProfile motionProfile;

  /** Motor values */
  private ElevatorPositionControllerValues motorValues = new ElevatorPositionControllerValues();

  /** Config for elevator mechanism */
  private final MechanismConfig config =
    MechanismBuilder.defaults()
      .motorConfig(
        MotorBuilder.defaults()
          .motorToMechRatio(7.5)
          .ccwPositive(false)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(1)
          .kI(0)
          .kD(0)
          .build())
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kA(0)
          .kG(0)
          .kS(0)
          .kV(0)
          .build())
      .motionProfileConfig(
        MotionProfileBuilder.defaults()
          .maxVelocity(3)
          .maxAcceleration(2)
          .build())
      .build();

  /** Initializes the elevator subsystem and configures hardware */
  private Elevator() {
    motor = ElevatorFactory.createDriveMotor(config);

    motionProfile = config.motionProfileConfig().createTrapezoidProfile();
  }

  /** Gets elevator subsystem instance if there is one, and creates one if there isn't */
  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }

    return instance;
  }

  @Override
  public void initializeTab() {
    // Get shuffleboard tab
    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

    // State info
    tab.add("Target state", new ElevatorStateSendable(() -> targetState));
    tab.addBoolean("At target state: ", () -> targetState == currentState);

    // Setpoint column
    ShuffleboardLayout setpointColumn = tab.getLayout("Setpoint", BuiltInLayouts.kList);

    setpointColumn.addDouble("Pos (m): ", () -> profiledSetpoint.position);
    setpointColumn.addDouble("Vel (mps): ", () -> profiledSetpoint.velocity);

    // Current position/velocity collumn
    ShuffleboardLayout stateColumn = tab.getLayout("Current state", BuiltInLayouts.kList);

    stateColumn.addDouble("Pos (m): ", () -> motorValues.posMeters);
    stateColumn.addDouble("Vel (mps): ", () -> motorValues.velMetersPerSec);
    stateColumn.addDouble("Acc (mpsps): ", () -> motorValues.accMetersPerSecPerSec);
  }

  @Override
  public void periodic() {
    motor.getUpdatedVals(motorValues);

    profiledSetpoint = calculateSetpoint(motorValues.posMeters, motorValues.velMetersPerSec, targetState);
    motor.setSetpoint(profiledSetpoint.position, profiledSetpoint.velocity);

    // update current state if safely reached target state
    if (Math.abs(motorValues.posMeters - targetState.getPosMeters()) < 0.01) {
      currentState = targetState;
    }

    motor.periodic();
  }

  /**
   * Get next setpoint according to trapezoidal motion profile
   * 
   * @param posMeters current position in meters
   * @param velMetersPerSec current velocity in meters
   * @param target target elevator state
   * @return new setpoint
   */
  public State calculateSetpoint(double posMeters, double velMetersPerSec, ElevatorState target) {
    return motionProfile.calculate(
      RobotConstants.PERIODIC_DURATION, 
      new State(posMeters, velMetersPerSec), 
      new State(target.getPosMeters(), target.getVelMetersPerSec()));
  }

  public Command stow() {
    return Commands.runOnce(
      () -> {
        targetState = ElevatorState.STOW;
      });
  }

  public Command l1() {
    return Commands.runOnce(
      () -> {
        targetState = ElevatorState.L1;
      });
  }

  public Command l2() {
    return Commands.runOnce(
      () -> {
        targetState = ElevatorState.L2;
      });
  }

  public Command l3() {
    return Commands.runOnce(
      () -> {
        targetState = ElevatorState.L3;
      });
  }

  public Command l4() {
    return Commands.runOnce(
      () -> {
        targetState = ElevatorState.L4;
      });
  }
}
