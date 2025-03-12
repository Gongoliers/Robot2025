package frc.robot.superstructure;

import java.util.function.BooleanSupplier;

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

  /** Superstructure Mechanism2d visualization */
  private SuperstructureMechanism mechanism;

  /** Initializes superstructure subsystem */
  private Superstructure() {
    elevator = Elevator.getInstance();
    pivot = Pivot.getInstance();
    intake = Intake.getInstance();

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
    Command movePivot = run(() -> pivot.setTargetState(targetState)).until(pivot::atTargetState);
    BooleanSupplier isSafe = () -> targetState.isSafe() || elevator.getState() == ElevatorState.STOW;
    return Commands.either(movePivot, Commands.none(), isSafe);
  };

  /**
   * Returns a command that moves the pivot to the safe position.
   *
   * @return a command that moves the pivot to the safe position.
   */
  private Command pivotToSafe() {
    return Commands.either(pivotTo(PivotState.SAFE), Commands.none(), pivot::isUnsafe);
  }

  /**
   * Returns a command that moves the elevator to a target state if the pivot is at a safe state
   * 
   * @param targetState target elevator state
   * @return a command that moves the elevator to a target state if the pivot is at a safe state
   */
  public Command elevatorTo(ElevatorState targetState) {
    Command moveElevator = run(() -> elevator.setTargetState(targetState)).until(elevator::atTargetState);
    return pivotToSafe().andThen(moveElevator);
  }

  /**
   * Returns a command that spins up the intake to a target state
   * 
   * @param targetState target intake state
   * @return a command that spins up the intake to a target state
   */
  public Command intakeTo(IntakeState targetState) {
      return run(() -> {
        intake.setTargetState(targetState);
      }).until(intake::atTargetState);
  }

  /**
   * Returns a command that moves the superstructure to some superstructure state
   * 
   * @param targetState target superstructure state
   * @return a command that moves the superstructure to some superstructure state
   */
  public Command superstructureTo(SuperstructureState targetState) {
    return elevatorTo(targetState.getElevatorState()).andThen(pivotTo(targetState.getPivotState()));
  }

  /**
   * Returns a command that intakes coral from the coral station
   * 
   * @return a command that intakes coral from the coral station
   */
  public Command intakeCoral() {
    Command toIntake = Commands.parallel(
            superstructureTo(SuperstructureState.INTAKE),
            intakeTo(IntakeState.CORALIN)
        );

    BooleanSupplier stopIntaking = () -> intake.beamBroken() || elevator.getState() == ElevatorState.STOW;

    Command toStow = Commands.parallel(
            superstructureTo(SuperstructureState.STOW),
            intakeTo(IntakeState.STOP)
        );

    BooleanSupplier stowed = () -> atTargetStates() && pivot.getState() == PivotState.STOW && elevator.getState() == ElevatorState.STOW;

    Command moveCoral = Commands.sequence(intakeTo(IntakeState.CORALOUT), Commands.waitSeconds(0.5), intakeTo(IntakeState.STOP));

    return toIntake.until(stopIntaking).andThen(toStow).until(stowed).andThen(moveCoral);
  }

  /**
   * Returns a command that automatically scores coral at some superstructure state
   * 
   * @param scoreState superstructure state to score at (only uses L1, L2, L3, L4)
   * @return a command that automatically scores coral at some superstructure state
   */
  public Command autoScore(SuperstructureState scoreState) {
    final Auto auto = Auto.getInstance();
    
    return superstructureTo(scoreState) //TODO: CLEAN UP THIS GODAWFUL COMMAND
      .alongWith(auto.pathfindToRecentTarget(0.1))
      .andThen(intakeTo(IntakeState.CORALIN))
      .andThen(Commands.waitSeconds(1.5))
      .andThen(auto.pathfindToRecentTarget(0.6)
        .alongWith(intakeTo(IntakeState.STOP))
        .alongWith(superstructureTo(SuperstructureState.STOW)));
  }
}
