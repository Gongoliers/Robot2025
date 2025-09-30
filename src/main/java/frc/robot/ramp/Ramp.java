package frc.robot.ramp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.Subsystem;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.controllers.velocity.VelocityController;
import frc.lib.controllers.velocity.VelocityController.VelocityControllerValues;
import frc.lib.sendables.RampStateSendable;

/** Ramp subsystem */
public class Ramp extends Subsystem {

  /** Ramp motor */
  private final VelocityController motor;

  /** Ramp motor values */
  private VelocityControllerValues motorValues = new VelocityControllerValues();

  /** Target state */
  private RampState targetState;

  /** Current state */
  private RampState currentState;

  /** Max difference between current rps and target rps for current state to be set to target state */
  private final double stateTolerance;

  /** Ramp subsystem config */
  private final MechanismConfig rampConfig =
    MechanismBuilder.defaults()
      .motorConfig(
        MotorBuilder.defaults()
          .ccwPositive(false)
          .motorToMechRatio(2)
          .statorCurrentLimit(50)
          .build())
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kA(0.0)
          .kS(0.512)
          .kV(0.22)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(0.25)
          .kI(0.0)
          .kD(0.0)
          .build())
      .build();

  public Ramp() {
    motor = RampFactory.createRampMotor(rampConfig);
    motor.configure();
    
    targetState = RampState.STOP;
    currentState = RampState.STOP;

    stateTolerance = 2;
  }

  @Override
  public void initializeTab() {
    // Get tab
    ShuffleboardTab tab = Shuffleboard.getTab("Ramp");

    // State info
    tab.add("Target state", new RampStateSendable(() -> targetState));
    tab.add("Current state", new RampStateSendable(() -> currentState));
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
    if (MathUtil.isNear(targetState.getVelRotationsPerSec(), motorValues.velRotationsPerSec, stateTolerance)) {
      currentState = targetState;
    } else {
      currentState = RampState.NONE;
    }

    motor.periodic();
  }

  public double getVelRotationsPerSec() {
    return motorValues.velRotationsPerSec;
  }

  public RampState getState() {
    return currentState;
  }

  public RampState getTargetState() {
    return targetState;
  }

  public void setTargetState(RampState state) {
    targetState = state;
  }

  public boolean atTargetState() {
    return currentState == targetState;
  }
}
