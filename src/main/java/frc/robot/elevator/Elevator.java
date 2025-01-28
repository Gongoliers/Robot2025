package frc.robot.elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.CAN;
import frc.lib.Subsystem;
import frc.lib.configs.FeedforwardControllerConfig;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MotionProfileConfig;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotionProfileConfig.MotionProfileBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.controllers.position.ElevatorPositionController;
import frc.robot.RobotConstants;

/** Elevator subsystem */
public class Elevator extends Subsystem {

  /** Elevator singleton */
  private static Elevator instance = null;

  /** Motor(s) that drives the elevator */
  private final ElevatorPositionController motor;

  /** Current elevator state */
  private ElevatorState state = ElevatorState.STOW;

  private final TrapezoidProfile motionProfile;

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
  public void initializeTab() {}

  @Override
  public void periodic() {
    motor.periodic();

    State profiledSetpoint = calculateSetpoint(motor.getElevatorPos(), 0, state);
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

      });
  }
}
