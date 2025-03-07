package frc.robot.superstructure;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.robot.auto.Auto;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorState;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.pivot.Pivot;
import frc.robot.pivot.PivotState;

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

  /** Auto reference */
  private final Auto auto;

  /** Superstructure Mechanism2d visualization */
  private SuperstructureMechanism mechanism;

  /** Initializes superstructure subsystem */
  private Superstructure() {
    elevator = Elevator.getInstance();
    pivot = Pivot.getInstance();
    intake = Intake.getInstance();
    auto = Auto.getInstance();

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
    return Commands.runOnce(() -> {
      if (elevator.getState() == ElevatorState.STOW || targetState.isSafe()) {
        pivot.setTargetState(targetState);
      }
    }).andThen(Commands.waitUntil(pivot::atTargetState));
  };

  /**
   * Returns a command that moves the elevator to a target state if the pivot is at a safe state
   * 
   * @param targetState target elevator state
   * @return a command that moves the elevator to a target state if the pivot is at a safe state
   */
  public Command elevatorTo(ElevatorState targetState) {
    return Commands
      .runOnce(() -> {
        if (pivot.getState().isUnsafe()) {
          pivot.setTargetState(PivotState.SCORE);
        }
      }).andThen(Commands.waitUntil(elevator::atTargetState))
      .andThen(
        () -> {
          elevator.setTargetState(targetState);
        }
      ).andThen(Commands.waitUntil(elevator::atTargetState));
  }

  /**
   * Returns a command that spins up the intake to a target state
   * 
   * @param targetState target intake state
   * @return a command that spins up the intake to a target state
   */
  public Command intakeTo(IntakeState targetState) {
    return Commands
      .runOnce(() -> {
        intake.setTargetState(targetState);
      });
  }

  /**
   * Returns a command that moves the superstructure to some superstructure state
   * 
   * @param targetState target superstructure state
   * @return a command that moves the superstructure to some superstructure state
   */
  public Command superstructureTo(SuperstructureState targetState) {
    return elevatorTo(targetState.getElevatorState())
      .andThen(pivotTo(targetState.getPivotState()));
  }

  /**
   * Returns a command that intakes coral from the coral station
   * 
   * @return a command that intakes coral from the coral station
   */
  public Command intakeCoral() {
    return superstructureTo(SuperstructureState.INTAKE)
      .alongWith(intakeTo(IntakeState.CORALIN))
      .andThen(Commands.waitUntil(intake::beamBroken))
      .andThen(superstructureTo(SuperstructureState.STOW))
      .alongWith(intakeTo(IntakeState.STOP));
  }

  /**
   * Returns a command that automatically scores coral at some superstructure state
   * 
   * @param scoreState superstructure state to score at (only uses L1, L2, L3, L4)
   * @return a command that automatically scores coral at some superstructure state
   */
  public Command autoScore(SuperstructureState scoreState) {
    return superstructureTo(scoreState) //TODO: CLEAN UP THIS GODAWFUL COMMAND
      .alongWith(auto.pathfindToRecentTarget(0.1))
      .andThen(intakeTo(IntakeState.CORALIN))
      .andThen(Commands.waitSeconds(1.5))
      .andThen(auto.pathfindToRecentTarget(0.6)
        .alongWith(intakeTo(IntakeState.STOP))
        .alongWith(superstructureTo(SuperstructureState.STOW)));
  }
}
