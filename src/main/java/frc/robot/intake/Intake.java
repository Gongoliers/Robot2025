package frc.robot.intake;

import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.controllers.velocity.VelocityController;
import frc.lib.controllers.velocity.VelocityController.VelocityControllerValues;
import frc.lib.sendables.IntakeStateSendable;

/** Intake subsystem */
public class Intake extends Subsystem {
  
  /** Intake singleton */
  private static Intake instance = null;

  /** Intake motor */
  private final VelocityController motor;

  /** Intake motor values */
  private VelocityControllerValues motorValues;

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
  }

  @Override
  public void periodic() {
    // Approach setpoint
    motor.getUpdatedVals(motorValues);

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
