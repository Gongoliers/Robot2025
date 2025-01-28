package frc.robot.elevator;

import frc.lib.CAN;
import frc.lib.Subsystem;
import frc.lib.configs.FeedforwardControllerConfig;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.controllers.position.LinearPositionController;

/** Elevator subsystem */
public class Elevator extends Subsystem {

  /** Elevator singleton */
  private static Elevator instance = null;

  /** Motor(s) that drives the elevator */
  private final LinearPositionController motor;

  /** Current elevator state */
  private ElevatorState state = ElevatorState.STOW;

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
      .build();

  /** Initializes the elevator subsystem and configures hardware */
  private Elevator() {
    motor = ElevatorFactory.createDriveMotor(
        new CAN(10),
        new CAN(11),
        config);
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
  }
}
