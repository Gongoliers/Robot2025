package frc.robot.manipulator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotionProfileConfig.MotionProfileBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.controllers.position.PositionController;
import frc.lib.controllers.position.PositionController.PositionControllerValues;
import frc.lib.controllers.velocity.VelocityController;
import frc.lib.controllers.velocity.VelocityController.VelocityControllerValues;
import frc.lib.sendables.IntakeStateSendable;
import frc.lib.sendables.PivotStateSendable;
import frc.robot.RobotConstants;

/** Manipulator subsystem */
public class Manipulator extends Subsystem {
  
  /** Manipulator singleton */
  private static Manipulator instance = null;

  /** Motors */
  private final PositionController pivotMotor;
  private final VelocityController intakeMotor;

  /** Motor values */
  private PositionControllerValues pivotValues = new PositionControllerValues();
  private VelocityControllerValues intakeValues = new VelocityControllerValues();

  /** Motion profile */
  private final TrapezoidProfile pivotProfile;

  /** Target states */
  private PivotState targetPivotState;
  private IntakeState targetIntakeState;

  private PivotState currentPivotState;
  private IntakeState currentIntakeState;

  /** Motor setpoints */
  private TrapezoidProfile.State profiledPivotSetpoint;
  private double setpointVelRotationsPerSec;

  /** Ideal pivot position */
  private double idealPivotPosRotations;
  private double idealPivotVelRotationsPerSec;

  /** Pivot motor config */
  private final MechanismConfig pivotConfig =
    MechanismBuilder.defaults()
      .motorConfig(
        MotorBuilder.defaults()
          .ccwPositive(true)
          .motorToMechRatio((58/10)*(58/18)*(30/12))
          .statorCurrentLimit(50)
          .build())
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kA(0.0)
          .kG(0.2145)
          .kS(0.0285)
          .kV(0.0)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(0.0)
          .kI(0.0)
          .kD(0.0)
          .build())
      .motionProfileConfig(
        MotionProfileBuilder.defaults()
          .maxVelocity(1)
          .maxAcceleration(1)
          .build())
      .build();

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
          .kS(0.0)
          .kV(0.0)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(0.0)
          .kI(0.0)
          .kD(0.0)
          .build())
      .build();

  /** Initializes the Manipulator subsystem and configures hardware */
  private Manipulator() {
    pivotMotor = ManipulatorFactory.createPivotMotor(pivotConfig);
    intakeMotor = ManipulatorFactory.createIntakeMotor(intakeConfig);

    pivotMotor.configure();
    intakeMotor.configure();

    pivotProfile = pivotConfig.motionProfileConfig().createTrapezoidProfile();

    targetPivotState = PivotState.STOW;
    targetIntakeState = IntakeState.STOP;

    currentPivotState = PivotState.STOW;
    currentIntakeState = IntakeState.STOP;

    profiledPivotSetpoint = new TrapezoidProfile.State();
    setpointVelRotationsPerSec = 0.0;

    idealPivotPosRotations = 0.0;
    idealPivotVelRotationsPerSec = 0.0;

    pivotMotor.setPos(-0.33203125);
  }

  /** Gets manipulator subsystem instance if there is one, and creates one if there isn't */
  public static Manipulator getInstance() {
    if (instance == null) {
      instance = new Manipulator();
    }

    return instance;
  }

  @Override
  public void initializeTab() {
    // Get shuffleboard tab
    ShuffleboardTab tab = Shuffleboard.getTab("Manipulator");

    // State info
    tab.add("Target pivot state", new PivotStateSendable(() -> targetPivotState));
    tab.add("Target intake state", new IntakeStateSendable(() -> targetIntakeState));
    tab.addBoolean("At target pivot state", () -> targetPivotState == currentPivotState);
    tab.addBoolean("At target intake state", () -> targetIntakeState == currentIntakeState);

    // Pivot setpoint column
    ShuffleboardLayout pivotSetpointColumn = tab.getLayout("Pivot setpoint", BuiltInLayouts.kList);

    pivotSetpointColumn.addDouble("Pos (rot)", () -> profiledPivotSetpoint.position);
    pivotSetpointColumn.addDouble("Vel (rps)", () -> profiledPivotSetpoint.velocity);

    // Current intake position/velocity column
    ShuffleboardLayout pivotStateColumn = tab.getLayout("Current pivot state", BuiltInLayouts.kList);

    pivotStateColumn.addDouble("Pos (rot)", () -> pivotValues.posRotations);
    pivotStateColumn.addDouble("Vel (rps)", () -> pivotValues.velRotationsPerSec);
    pivotStateColumn.addDouble("Acc (rpsps)", () -> pivotValues.accRotationsPerSecPerSec);
    pivotStateColumn.addDouble("Volts", () -> pivotValues.motorVolts);
    pivotStateColumn.addDouble("Amps", () -> pivotValues.motorAmps);

    // Ideal intake position/velocity column
    ShuffleboardLayout idealPivotStateColumn = tab.getLayout("Ideal pivot state", BuiltInLayouts.kList);

    idealPivotStateColumn.addDouble("Ideal pos (rot)", () -> idealPivotPosRotations);
    idealPivotStateColumn.addDouble("Ideal vel (rps)", () -> idealPivotVelRotationsPerSec);

    // Intake setpoint column
    ShuffleboardLayout intakeSetpointColumn = tab.getLayout("Intake setpoint", BuiltInLayouts.kList);

    intakeSetpointColumn.addDouble("Vel (rps)", () -> setpointVelRotationsPerSec);
    
    // Current intake velocity column
    ShuffleboardLayout intakeStateColumn = tab.getLayout("Current intake state", BuiltInLayouts.kList);

    intakeStateColumn.addDouble("Vel (rps)", () -> intakeValues.velRotationsPerSec);
    intakeStateColumn.addDouble("Acc (rpsps)", () -> intakeValues.accRotationsPerSecPerSec);
    intakeStateColumn.addDouble("Volts", () -> intakeValues.motorVolts);
    intakeStateColumn.addDouble("Amps", () -> intakeValues.motorAmps);
  }

  @Override
  public void periodic() {
    // approach setpoints
    pivotMotor.getUpdatedVals(pivotValues);
    intakeMotor.getUpdatedVals(intakeValues);

    profiledPivotSetpoint = calculateSetpoint();
    idealPivotPosRotations = profiledPivotSetpoint.position;
    idealPivotVelRotationsPerSec = profiledPivotSetpoint.velocity;

    pivotMotor.setSetpoint(profiledPivotSetpoint.position, profiledPivotSetpoint.velocity);
    intakeMotor.setSetpoint(setpointVelRotationsPerSec);

    // update current states if safely reached target states
    if (Math.abs(pivotValues.posRotations - targetPivotState.getPosRotations()) < 0.01) {
      currentPivotState = targetPivotState;
    } else {
      currentPivotState = null;
    }

    if (Math.abs(intakeValues.velRotationsPerSec - targetIntakeState.getVelRotationsPerSec()) < 5) {
      currentIntakeState = targetIntakeState;
    } else {
      currentIntakeState = null;
    }

    pivotMotor.periodic();
    intakeMotor.periodic();
  }

  private TrapezoidProfile.State calculateSetpoint() {
    return pivotProfile.calculate(
      RobotConstants.PERIODIC_DURATION, 
      new TrapezoidProfile.State(idealPivotPosRotations, idealPivotVelRotationsPerSec), 
      new TrapezoidProfile.State(targetPivotState.getPosRotations(), targetPivotState.getVelRotationsPerSec()));
  }

  public double getPosRotations() {
    return pivotValues.posRotations;
  }

  public void setTargetPivotState(PivotState state) {
    targetPivotState = state;
  }

  public void setTargetIntakeState(IntakeState state) {
    targetIntakeState = state;
  }

  public boolean atTargetPivotState() {
    return targetPivotState == currentPivotState;
  }

  public boolean atTargetIntakeState() {
    return targetIntakeState == currentIntakeState;
  }

  public Command zeroPivot() {
    return Commands.runOnce(
      () -> {
        pivotMotor.setPos(0.33203125);
      });
  }
}
