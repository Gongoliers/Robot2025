package frc.robot.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.UnitsHelper;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotionProfileConfig.MotionProfileBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.controllers.ClosedLoopPositionController;
import frc.lib.controllers.Controller;
import frc.lib.controllers.position.PositionController;
import frc.lib.outputs.ElevatorSimOutput;
import frc.lib.outputs.Output;
import frc.lib.values.MotorValues;

import static edu.wpi.first.units.Units.*;

public class Elevator extends Subsystem {

  /** Elevator subsystem singleton */
  private static Elevator instance = null;

  /** Elevator position controller */
  private final PositionController positionController;

  /** Elevator controller output */
  private final Output<Voltage, MotorValues> output;

  /** Elevator position controller values */
  private MotorValues positionControllerValues = new MotorValues();

  /** Ratio of meters travelled by elevator per rotations made by position controller */
  private static final Per<DistanceUnit, AngleUnit> ROT_TO_METER = Meters.of(0.02).per(Rotation);
  private static final Per<LinearVelocityUnit, AngularVelocityUnit> RPS_TO_MPS = UnitsHelper.velocityConversionFactor(ROT_TO_METER);
  private static final Per<LinearAccelerationUnit, AngularAccelerationUnit> RPSPS_TO_MPSPS= UnitsHelper.accelerationConversionFactor(ROT_TO_METER);

  private final Controller<Angle, MotorValues> positionController2;

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

    // NOTE: These values of kV and kA are placeholder data for the actual subsystem
    var kV_mps = MetersPerSecond.of(9.6).div(Volts.of(12));
    var kA_mpsps = MetersPerSecondPerSecond.of(228.13).div(Volts.of(12));

    var kV_rps = UnitsHelper.kV_rps(kV_mps, ROT_TO_METER);
    var kA_rpsps = UnitsHelper.kA_rpsps(kA_mpsps, ROT_TO_METER);

    output = new ElevatorSimOutput(UnitsHelper.inverseKv(kV_rps), UnitsHelper.inverseKa(kA_rpsps), DCMotor.getKrakenX60(2), true, UnitsHelper.inverseConversionFactor(ROT_TO_METER));
    positionController2 = new ClosedLoopPositionController(output, Volts.of(0.25).per(Rotation), Volts.of(0.01).per(RotationsPerSecond), () -> Volts.of(0.516));

    currentState = ElevatorState.STOW;
    targetState = ElevatorState.STOW;

    stateTolerance = Meters.of(0.02);

    motionProfile = config.motionProfileConfig().createTrapezoidProfile();
    profiledSetpoint = new TrapezoidProfile.State(0, 0);
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

    setpointColumn.addDouble("Setpoint position (m)", () -> targetState.getPosMeters());

    // Current position/velocity column
    ShuffleboardLayout stateColumn = tab.getLayout("Current state", BuiltInLayouts.kList);

    stateColumn.addString("Name", () -> currentState.name());
    stateColumn.addDouble("Elevator position (m)", () -> positionControllerValues.position.timesConversionFactor(ROT_TO_METER).in(Meters));
    stateColumn.addDouble("Elevator velocity (mps)", () -> positionControllerValues.velocity.timesConversionFactor(RPS_TO_MPS).in(MetersPerSecond));
    stateColumn.addDouble("Elevator acceleration (mpsps)", () -> positionControllerValues.acceleration.timesConversionFactor(RPSPS_TO_MPSPS).in(MetersPerSecondPerSecond));
    stateColumn.addDouble("Motor position (rot)", () -> positionControllerValues.position.in(Rotations));
    stateColumn.addDouble("Motor velocity (rps)", () -> positionControllerValues.velocity.in(RotationsPerSecond));
    stateColumn.addDouble("Motor acceleration (rpsps)", () -> positionControllerValues.acceleration.in(RotationsPerSecondPerSecond));;
    stateColumn.addDouble("Motor voltage",  () -> positionControllerValues.motorVoltage.in(Volts));
    stateColumn.addDouble("Stator current", () -> positionControllerValues.statorCurrent.in(Amps));
    stateColumn.addDouble("Supply current", () -> positionControllerValues.supplyCurrent.in(Amps));
  }

//  @Override
//  public void periodic() {
//
//  }

  @Override
  public void periodic() {
    positionControllerValues = positionController2.getOutputValues();

    Distance position = positionControllerValues.position.timesConversionFactor(ROT_TO_METER);

    if (MathUtil.isNear(targetState.getPosMeters(), position.in(Meters), stateTolerance.in(Meters))) {
      // If close enough to target state, consider the eleevator to be at that state
      currentState = targetState;
    } else {
      // If not, conisder hteelevator to be moving
      currentState = ElevatorState.MOVING;
    }

    if (targetState == ElevatorState.STOW && MathUtil.isNear(0.0, position.in(Meters), 0.04)) {
      // If near enough to stow position and you want to stow, disable the motors to prevent stalling
      output.update(Volts.of(0.01));
    } else {
      positionController2.update(Meters.of(targetState.getPosMeters()).timesConversionFactor(Rotations.of(50).per(Meter)));
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
