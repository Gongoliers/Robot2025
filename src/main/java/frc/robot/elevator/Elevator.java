package frc.robot.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private ElevatorState targetState;
  private ElevatorState currentState;

  /** Profiled elevator setpoint */
  private TrapezoidProfile.State profiledSetpoint;

  /** Trapezoidal motion profile for smooth movement from setpoint to setpoint */
  private final TrapezoidProfile motionProfile;

  /** Ideal current state, follows profile perfectly */
  private double idealPosMeters;
  private double idealVelMetersPerSec;

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
          .kG(0.329)
          .kS(0.111)
          .kV(3.1)
          .build())
      .motionProfileConfig(
        MotionProfileBuilder.defaults()
          .maxVelocity(3)
          .maxAcceleration(3)
          .build())
      .build();

  /** Initializes the elevator subsystem and configures hardware */
  private Elevator() {
    motor = ElevatorFactory.createDriveMotor(config);
    motor.configure();
    motor.setElevatorPos(0.0);

    targetState = ElevatorState.STOW;
    currentState = ElevatorState.STOW;

    profiledSetpoint = new TrapezoidProfile.State();

    idealPosMeters = 0.0;
    idealVelMetersPerSec = 0.0;

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
    tab.addBoolean("At target state", () -> targetState == currentState);

    // Setpoint column
    ShuffleboardLayout setpointColumn = tab.getLayout("Setpoint", BuiltInLayouts.kList);

    setpointColumn.addDouble("Pos (m)", () -> profiledSetpoint.position);
    setpointColumn.addDouble("Vel (mps)", () -> profiledSetpoint.velocity);

    // Current position/velocity column
    ShuffleboardLayout stateColumn = tab.getLayout("Current state", BuiltInLayouts.kList);

    stateColumn.addDouble("Pos (m)", () -> motorValues.posMeters);
    stateColumn.addDouble("Vel (mps)", () -> motorValues.velMetersPerSec);
    stateColumn.addDouble("Acc (mpsps)", () -> motorValues.accMetersPerSecPerSec);
    stateColumn.addDouble("Volts",  () -> motorValues.motorVolts);
    stateColumn.addDouble("Amps", () -> motorValues.motorAmps);

    // Ideal position/velocity column
    ShuffleboardLayout idealStateColumn = tab.getLayout("Ideal state", BuiltInLayouts.kList);

    idealStateColumn.addDouble("Ideal pos (m)", () -> idealPosMeters);
    idealStateColumn.addDouble("Ideal vel (mps)", () -> idealVelMetersPerSec);
  }

  @Override
  public void periodic() {
    motor.getUpdatedVals(motorValues);

    profiledSetpoint = calculateSetpoint(idealPosMeters, idealVelMetersPerSec, targetState);
    idealPosMeters = profiledSetpoint.position;
    idealVelMetersPerSec = profiledSetpoint.velocity;

    motor.setSetpoint(profiledSetpoint.position, profiledSetpoint.velocity);

    // update current state if safely reached target state
    if (Math.abs(motorValues.posMeters - targetState.getPosMeters()) < 0.01) {
      currentState = targetState;
    } else {
      currentState = null;
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
  public TrapezoidProfile.State calculateSetpoint(double posMeters, double velMetersPerSec, ElevatorState target) {
    return motionProfile.calculate(
      RobotConstants.PERIODIC_DURATION, 
      new TrapezoidProfile.State(posMeters, velMetersPerSec), 
      new TrapezoidProfile.State(target.getPosMeters(), target.getVelMetersPerSec()));
  }

  public void setTargetState(ElevatorState state) {
    targetState = state;
  }

  public double getPosMeters() {
    return motorValues.posMeters;
  }

  public boolean atTargetState() {
    return targetState == currentState;
  }

  public Command zero() {
    return Commands.runOnce(
      () -> {
        motor.setElevatorPos(0);
      });
  }
}
