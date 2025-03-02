package frc.robot.intake;

import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.controllers.velocity.VelocityController;
import frc.lib.controllers.velocity.VelocityController.VelocityControllerValues;
import frc.lib.sendables.IntakeStateSendable;
import frc.lib.sensors.TimeOfFlight;
import frc.lib.sensors.TimeOfFlight.TimeOfFlightValues;

/** Intake subsystem */
public class Intake extends Subsystem {
  
  /** Intake singleton */
  private static Intake instance = null;

  /** Intake motor */
  private final VelocityController motor;

  /** Time of flight sensor used as beam break */
  private final TimeOfFlight timeOfFlight;

  /** Intake motor values */
  private VelocityControllerValues motorValues;

  /** Time of flight sensor values */
  private TimeOfFlightValues timeOfFlightValues;

  /** Target state */
  private IntakeState targetState;

  /** Current state */
  private IntakeState currentState;

  /** Max difference between current rps and target rps for current state to be set to target state */
  private final double stateTolerance;

  /** Intake subsystem config */
  private final MechanismConfig intakeConfig =
    MechanismBuilder.defaults()
      .motorConfig(
        MotorBuilder.defaults()
          .ccwPositive(false)
          .motorToMechRatio(32/12)
          .statorCurrentLimit(20)
          .build())
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kA(0.0)
          .kS(0.161)
          .kV(0.25)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(0.2)
          .kI(0.0)
          .kD(0.0)
          .build())
      .build();

  /** Gets intake subsystem instance */
  public static Intake getInstance() {
    if (instance == null) {
      instance = new Intake();
    }

    return instance;
  }

  /** Initialize intake subsystem */
  private Intake() {
    motor = IntakeFactory.createIntakeMotor(intakeConfig);
    motor.configure();

    timeOfFlight = IntakeFactory.createTimeOfFlightSensor();
    timeOfFlight.configure();
    timeOfFlight.setBeambreakThreshold(0.2);
    timeOfFlight.beamBroken().onTrue(Commands.runOnce(() -> setTargetState(IntakeState.STOP)));

    targetState = IntakeState.STOP;
    currentState = IntakeState.STOP;
    stateTolerance = 2;
  }

  @Override
  public void initializeTab() {
    // Get tab
    ShuffleboardTab tab = Shuffleboard.getTab("Intake");

    // State info
    tab.add("Target state", new IntakeStateSendable(() -> targetState));
    tab.add("Current state", new IntakeStateSendable(() -> currentState));
    tab.addBoolean("At target state", () -> targetState == currentState);

    // Current values column
    ShuffleboardLayout values = tab.getLayout("Current values", BuiltInLayouts.kList);

    values.addDouble("Vel (rotps)", () -> motorValues.velRotationsPerSec);
    values.addDouble("Acc (rotpsps)", () -> motorValues.accRotationsPerSecPerSec);
    values.addDouble("Voltage", () -> motorValues.motorVolts);
    values.addDouble("Current", () -> motorValues.motorAmps);

    // Time of flight values column
    ShuffleboardLayout tofValues = tab.getLayout("Time of flight values", BuiltInLayouts.kList);

    tofValues.addDouble("Distance (m)", () -> timeOfFlightValues.distanceMeters);
    tofValues.addBoolean("Beam broken", () -> timeOfFlightValues.beamBroken);
  }

  @Override
  public void periodic() {
    // Approach setpoint
    motor.getUpdatedVals(motorValues);
    timeOfFlight.getUpdatedVals(timeOfFlightValues);

    motor.setSetpoint(targetState.getVelRotationsPerSec());

    // Update current state if close enough to target state
    if (Math.abs(motorValues.velRotationsPerSec - targetState.getVelRotationsPerSec()) <= stateTolerance) {
      currentState = targetState;
    } else {
      currentState = IntakeState.NONE;
    }

    motor.periodic();
  }

  public double getVelRotationsPerSec() {
    return motorValues.velRotationsPerSec;
  }

  public IntakeState getState() {
    return currentState;
  }

  public IntakeState getTargetState() {
    return targetState;
  }

  public void setTargetState(IntakeState state) {
    targetState = state;
  }

  public boolean atTargetState() {
    return currentState == targetState;
  }
}
