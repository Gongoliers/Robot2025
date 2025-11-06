package frc.robot.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
import frc.lib.motors.MotorOutput;
import frc.lib.motors.MotorValues;
import frc.robot.RobotConstants;

public class Elevator extends MultithreadedSubsystem {

  /** Elevator subsystem singleton */
  private static Elevator instance = null;

  // Motor output related

  /** Elevator motor output */
  private final MotorOutput motorOutput;

  /** Elevator motor output values */
  private MotorValues motorValues = new MotorValues();

  /** Elevator position offset to allow elevator position to be rezeroed */
  private Distance positionOffset = Meters.of(0.0);

  /** Ratio of meters travelled by elevator per rotations made by motor output */
  private final double rotationsToMeters;

  // State related

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

  /** If true, a voltage is manually set, otherwise, false */
  private boolean voltageSet;

  // Motor control related

  /** Voltage to set motor output */
  private MutVoltage voltageOut;

  /** PID controller for feedback control */
  private PIDController feedbackController;
  
  /** Feedforward controller for feedforward control */
  private ElevatorFeedforward feedforwardController;

  /** Determines how close to zero target velocity (target velocity being the velcoity calculated by the motion profile) before PID starts assisting with final positioning */
  private LinearVelocity PIDThreshold;

  /** Determines how smoothly feedback voltage begins assisting (0 -> instant, (0, 1) -> snappy, 1 -> linear, (1, inf), smooth) (this is an exponential interpolation, smoothing is the exponent used) */
  private double PIDSmoothing;

  /** Mechanism config */
  private final MechanismConfig config = MechanismBuilder.defaults()
    .feedforwardControllerConfig(FeedforwardControllerBuilder.defaults()
      .kV(0.1)
      .kA(0.09)
      .kG(0.575)
      .kS(0.12)
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
      .rotorToSensorRatio(5.0)
      .sensorToMechRatio(5.0)
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
    motorOutput = ElevatorFactory.createMotorOutput(config);

    rotationsToMeters = 0.031 * Math.PI * 3;

    currentState = ElevatorState.STOW;
    targetState = ElevatorState.STOW;

    stateTolerance = Meters.of(0.01);

    motionProfile = config.motionProfileConfig().createTrapezoidProfile();
    profiledSetpoint = new TrapezoidProfile.State(targetState.getPosMeters(), 0);

    voltageOut = Volts.mutable(0.0);
    feedbackController = config.feedbackControllerConfig().createPIDController();
    feedforwardController = config.feedforwardControllerConfig().createElevatorFeedforward();
    PIDThreshold = MetersPerSecond.of(0.1);
    PIDSmoothing = 2;
  }

  @Override
  public void initializeTab() {
    // Get shuffleboard tab
    ShuffleboardTab tab = Shuffleboard.getTab("Elevator");

    // State info
    tab.addString("Target state", () -> targetState.name());
    tab.addBoolean("At target state", () -> targetState == currentState);

    // Setpoint column
    ShuffleboardLayout setpointColumn = tab.getLayout("Setpoint", BuiltInLayouts.kList);

    setpointColumn.addDouble("Setpoint position (m)", () -> profiledSetpoint.position);
    setpointColumn.addDouble("Setpoint velocity (m/s)", () -> profiledSetpoint.velocity);

    // Current state column
    ShuffleboardLayout stateColumn = tab.getLayout("Current state", BuiltInLayouts.kList);

    stateColumn.addDouble("Elevator position (m)", () -> motorValues.position.in(Rotations) * rotationsToMeters + positionOffset.in(Meters));
    stateColumn.addDouble("Elevator velocity (m/s)", () -> motorValues.velocity.in(RotationsPerSecond) * rotationsToMeters + positionOffset.in(Meters));
    stateColumn.addDouble("Elevator acceleration (m/s/s)", () -> motorValues.acceleration.in(RotationsPerSecondPerSecond) * rotationsToMeters + positionOffset.in(Meters));
    stateColumn.addDouble("Motor position (rot)", () -> motorValues.position.in(Rotations));
    stateColumn.addDouble("Motor velocity (rot/s)", () -> motorValues.velocity.in(RotationsPerSecond));
    stateColumn.addDouble("Motor acceleration (rot/s/s)", () -> motorValues.acceleration.in(RotationsPerSecondPerSecond));
    stateColumn.addDouble("Motor voltage",  () -> motorValues.motorVoltage.in(Volts));
    stateColumn.addDouble("Stator current", () -> motorValues.statorCurrent.in(Amps));
    stateColumn.addDouble("Supply current", () -> motorValues.supplyCurrent.in(Amps));
  }
  
  @Override
  public void periodic() {

  }

  @Override
  public void fastPeriodic() {
    motorOutput.updateValues(motorValues, Seconds.of(RobotConstants.FAST_PERIODIC_DURATION));

    Distance position = Meters.of(motorValues.position.in(Rotations) * rotationsToMeters).plus(positionOffset);

    if (MathUtil.isNear(targetState.getPosMeters(), position.in(Meters), stateTolerance.in(Meters))) {
      // If close enough to target state, consider the eleevator to be at that state
      currentState = targetState;
    } else {
      // If not, conisder hteelevator to be moving
      currentState = ElevatorState.MOVING;
    }

    if (targetState == ElevatorState.STOW && position.in(Meters) < 0.01) {
      // If near enough to stow position and you want to stow, disable the motors to prevent stalling
      voltageOut.mut_replace(0.1, Volts);
      voltageSet = true;
      currentState = ElevatorState.STOW;
    }

    if (currentState != targetState) {
      // If not at target state yet, approach state with motion profile
      profiledSetpoint = motionProfile.calculate(
          RobotConstants.FAST_PERIODIC_DURATION, 
          profiledSetpoint, 
          new TrapezoidProfile.State(targetState.getPosMeters(), 0));
    } else {
      // If reached target state, set setpont to hold at that state
      profiledSetpoint = new TrapezoidProfile.State(targetState.getPosMeters(), 0.0);
    }

    if (voltageSet == false) {
      // If no manual voltage set, calculate voltage using feedforward and feedback
      double feedforwardVolts = feedforwardController.calculate(profiledSetpoint.velocity);
      double feedbackVolts = 0.0;
      
      if (MathUtil.isNear(0.0, motorValues.velocity.in(RotationsPerSecond) * rotationsToMeters, PIDThreshold.in(MetersPerSecond))) {
        // If target velocity is close enough to zero, meaning you are reacing the end of a trajectory, fade in some feedback voltage
        feedbackVolts = feedbackController.calculate(motorValues.position.in(Rotations) * rotationsToMeters, profiledSetpoint.position);

        // Calculation to fade in PID voltage smoothly based on velocity's closeness to 0
        double t = Math.abs(motorValues.velocity.in(RotationsPerSecond) * rotationsToMeters)/PIDThreshold.in(MetersPerSecond); // This gives the value of current velocity as a percentage of PIDThreshold
        t = t * -1 + 1; // Inverting and adding 1 means now 0.0 refers to a velocity of PIDThreshold, and 1.0 refers to a velocity of 0; this could already be multiplied by PID voltage for a linear fade in
        t = Math.pow(t, PIDSmoothing); // This smooths the linear interpolation based on PIDSmoothing (to understand this go into desmos, graph x^a, and vary a. a is PIDSmoothing, and x is the value of t before this calculation)

        feedbackVolts *= t; // This is what does the fading in
      }

      voltageOut.mut_replace(feedforwardVolts + feedbackVolts, Volts);
    } else {
      // Otherwise, dont change voltageOut at all, and reset voltageSet
      voltageSet = false;
    }

    // Set motor output voltage
    motorOutput.setVoltage(voltageOut);
  }

  /**
   * Set position of the elevator for rezeroing
   * 
   * @param newPos new position of the elevator
   */
  private void setPosition(Distance newPos) {
    positionOffset = newPos.minus(Meters.of(motorValues.position.in(Rotations) * rotationsToMeters));
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

  public Command setElevatorPosition(Distance newPosition) {
    return Commands.runOnce(() -> {
      setPosition(newPosition);
    });
  }
}
