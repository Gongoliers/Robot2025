package frc.robot.superstructure;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

  /** Cancels all waits when true */
  private boolean cancelAll = false;

  /** Initializes superstructure subsystem */
  public Superstructure(Elevator elevator, Pivot pivot, Intake intake, Ramp ramp, Endgame endgame) {
    this.elevator = elevator;
    this.pivot = pivot;
    this.intake = intake;
    this.ramp = ramp;
    this.endgame = endgame;

    mechanism = new SuperstructureMechanism(elevator::getPosMeters, pivot::getPosRotations);
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
    return Commands.runOnce(() -> pivot.setTargetState(targetState))
      .andThen(Commands.waitUntil(() -> pivot.atTargetState() || cancelAll));
  };

  /**
   * Returns a command that moves the elevator to a target state if the pivot is at a safe state
   * 
   * @param targetState target elevator state
   * @return a command that moves the elevator to a target state if the pivot is at a safe state
   */
  public Command elevatorTo(ElevatorState targetState) {
    return Commands.runOnce(() -> elevator.setTargetState(targetState))
      .andThen(Commands.waitUntil(() -> elevator.atTargetState() || cancelAll));
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
    .andThen(Commands.either(
      pivotTo(targetState.getPivotState())
        .andThen(elevatorTo(targetState.getElevatorState())),
      elevatorTo(targetState.getElevatorState())
        .andThen(pivotTo(targetState.getPivotState())),
      () -> targetState.getElevatorState() == ElevatorState.STOW))
      .andThen(Commands.either(
        intakeTo(IntakeState.CORALOUTSLOW, RampState.STOP)
          .andThen(Commands.waitSeconds(0.1))
          .andThen(intakeTo(IntakeState.STOP, RampState.STOP)),
        Commands.none(),
        () -> targetState.getElevatorState() == ElevatorState.L4))
      .andThen(Commands.print("Done superstructure"));
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
      return intake.beamBroken() || this.targetState == SuperstructureState.STOW || cancelAll;
    })
    .andThen(Commands.either(
      Commands.waitSeconds(0.05)
        .andThen(intakeTo(IntakeState.STOP, RampState.STOP))
        .andThen(superstructureTo(SuperstructureState.HARDSTOP))
        .andThen(intakeTo(IntakeState.CORALOUTSLOW, RampState.STOP))
        .andThen(Commands.waitSeconds(0.3))
        .andThen(intakeTo(IntakeState.STOP, RampState.STOP))
        .andThen(intakeTo(IntakeState.CORALINSLOW, RampState.STOP))
        .andThen(Commands.waitSeconds(0.14))
        .andThen(intakeTo(IntakeState.STOP, RampState.STOP))
        .andThen(superstructureTo(SuperstructureState.STOW)), 
      superstructureTo(SuperstructureState.STOW)
        .andThen(intakeTo(IntakeState.STOP, RampState.STOP)), 
      () -> this.targetState != SuperstructureState.STOW || !this.cancelAll)));
  }

  public Command scoreCoral() {
    return Commands.either(
      intakeTo(IntakeState.CORALINFAST, RampState.STOP)
        .andThen(Commands.waitSeconds(0.3)), 
      intakeTo(IntakeState.CORALINSLOW, RampState.STOP)
        .andThen(Commands.waitSeconds(0.4)), 
      () -> this.targetState == SuperstructureState.L4)
        .andThen(intakeTo(IntakeState.STOP, RampState.STOP)
        .andThen(superstructureTo(SuperstructureState.STOW)));
  }
  
  public Command manualClimb(CommandXboxController controller) {
    return Commands.runOnce(() -> {
      endgame.setTargetState(EndgameState.ARMED);
    }).andThen(Commands.waitUntil(() -> endgame.atTargetState()))
    .andThen(Commands.run(() -> {
      if (endgame.getPosRotations() > 0.45) { // super professional code
        endgame.setVoltage(0.0);
      } else if (controller.rightTrigger().getAsBoolean()) {
        endgame.setVoltage(-2.0);
      } else if (controller.leftTrigger().getAsBoolean()) {
        endgame.setVoltage(2.0);
      } else {
        endgame.setVoltage(0.25);
      }
    }).until(() -> this.cancelAll));
  }

  /**
   * Returns a command that SHOULD force all wait commands to stop waiting
   */
  public Command cancelAll() {
    return Commands.runOnce(() -> {
      cancelAll = true;
    }).andThen(
      () -> {
        cancelAll = false;
      }
    );
  }
}
