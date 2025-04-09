package frc.robot.superstructure;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.robot.auto.Auto;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorState;
import frc.robot.endgame.Endgame;
import frc.robot.endgame.EndgameState;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.pivot.Pivot;
import frc.robot.pivot.PivotState;
import frc.robot.ramp.Ramp;
import frc.robot.ramp.RampState;

/** Superstructure subsystem */
public class Superstructure extends Subsystem {
  
  /** Superstructure subsystem singleton */
  private static Superstructure instance = null;

  /** Elevator reference */
  private final Elevator elevator;

  /** Pivot reference */
  private final Pivot pivot;

  /** Intake reference */
  private final Intake intake;

  /** Ramp reference */
  private final Ramp ramp;

  /** Endgame reference */
  private final Endgame endgame;

  /** Target superstructre state (for cancelling things mostly) */
  private SuperstructureState targetState = SuperstructureState.STOW;

  /** Superstructure Mechanism2d visualization */
  private SuperstructureMechanism mechanism;

  /** Initializes superstructure subsystem */
  private Superstructure() {
    elevator = Elevator.getInstance();
    pivot = Pivot.getInstance();
    intake = Intake.getInstance();
    ramp = Ramp.getInstance();
    endgame = Endgame.getInstance();

    mechanism = new SuperstructureMechanism(elevator::getPosMeters, pivot::getPosRotations);
  }

  /**
   * Gets superstructure subsystem instance if there is one, and creates and returns one if there isn't
   * 
   * @return superstructure subsystem instance
   */
  public static Superstructure getInstance() {
    if (instance == null) {
      instance = new Superstructure();
    }

    return instance;
  }

  @Override
  public void periodic() {
    mechanism.periodic();
  }

  @Override
  public void initializeTab() {
    // Get shuffleboard tab
    ShuffleboardTab tab = Shuffleboard.getTab("Superstructure");

    // Mechanism2d visualizer
    tab.add("Superstructure", mechanism.getSendable());

    // Everything at their target state
    tab.addBoolean("At target states", () -> atTargetStates());
  }

  /**
   * Returns true if all subsystems have reached their target states
   * 
   * @return true if all subsystems have reached their target states
   */
  public boolean atTargetStates() {
    return 
      pivot.atTargetState() &&
      intake.atTargetState() &&
      elevator.atTargetState();
  }

  /**
   * Returns a command that moves the pivot to a target state if the elevator is stowed
   * 
   * @param targetState target pivot state
   * @return a command that moves the pivot to a target state if the elevator is stowed
   */
  public Command pivotTo(PivotState targetState) {
    return Commands.runOnce(() -> pivot.setTargetState(targetState));
  };

  /**
   * Returns a command that moves the elevator to a target state if the pivot is at a safe state
   * 
   * @param targetState target elevator state
   * @return a command that moves the elevator to a target state if the pivot is at a safe state
   */
  public Command elevatorTo(ElevatorState targetState) {
    return Commands.runOnce(() -> elevator.setTargetState(targetState));
  }

  /**
   * Returns a command that spins up the intake to a target state
   * 
   * @param targetState target intake state
   * @return a command that spins up the intake to a target state
   */
  public Command intakeTo(IntakeState targetIntakeState, RampState targetRampState) {
    return Commands.runOnce(() -> {
      intake.setTargetState(targetIntakeState);
      ramp.setTargetState(targetRampState);
    });
  }

  /**
   * Returns a command that moves the superstructure to some superstructure state
   * 
   * @param targetState target superstructure state
   * @return a command that moves the superstructure to some superstructure state
   */
  public Command superstructureTo(SuperstructureState targetState) {
    return Commands.runOnce(() -> this.targetState = targetState)
    .andThen(elevatorTo(targetState.getElevatorState()))
    .andThen(pivotTo(targetState.getPivotState()))
    .andThen(Commands.waitUntil(() -> {
      return elevator.atTargetState() && pivot.atTargetState();
    }))
    .andThen(Commands.either(
      intakeTo(IntakeState.CORALOUT, RampState.STOP)
      .andThen(Commands.waitSeconds(0.1))
      .andThen(intakeTo(IntakeState.STOP, RampState.STOP)), 
      Commands.none(), () -> targetState.getElevatorState() == ElevatorState.L4));
  }

  /**
   * Returns a command that intakes coral from the coral station
   * 
   * @return a command that intakes coral from the coral station
   */
  public Command intakeCoral() {
    return superstructureTo(SuperstructureState.INTAKE)
    .andThen(intakeTo(IntakeState.CORALIN, RampState.INTAKEFAST))
    .andThen(Commands.waitUntil(() -> {
      return intake.beamBroken() || this.targetState == SuperstructureState.STOW;
    })
    .andThen(superstructureTo(SuperstructureState.STOW))
    .andThen(intakeTo(IntakeState.STOP, RampState.STOP)));
  }

  public Command scoreCoral() {
    return Commands.either(
      intakeTo(IntakeState.CORALINFAST, RampState.STOP)
        .andThen(Commands.waitSeconds(0.3)), 
      intakeTo(IntakeState.CORALINSLOW, RampState.STOP)
        .andThen(Commands.waitSeconds(0.4)), 
      () -> this.targetState == SuperstructureState.L4)
        .andThen(intakeTo(IntakeState.STOP, RampState.STOP)
        .alongWith(superstructureTo(SuperstructureState.STOW)));
  }
  
  public Command climb() {
    return Commands.runOnce(() -> {
      endgame.setTargetState(EndgameState.ARMED);
    }).andThen(Commands.waitUntil(() -> endgame.atTargetState()))
    .andThen(Commands.run(() -> {
      if (endgame.getPosRotations() < 0.4) {
        endgame.setVoltage(0.3);
      } else {
        endgame.setVoltage(0.0);
      }
    }));
  }
}
