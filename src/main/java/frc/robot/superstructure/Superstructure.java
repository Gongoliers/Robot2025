package frc.robot.superstructure;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorState;
import frc.robot.manipulator.Manipulator;
import frc.robot.manipulator.PivotState;

/** Superstructure subsystem */
public class Superstructure extends Subsystem {
  
  /** Superstructure subsystem singleton */
  private static Superstructure instance = null;

  /** Elevator reference */
  private final Elevator elevator;

  /** Manipulator reference */
  private final Manipulator manipulator;

  /** Requested pivot state (ignores moving to a safe position for movement) */
  private PivotState requestedPivotState;

  /** Superstructure Mechanism2d visualization */
  private SuperstructureMechanism mechanism;

  /** Initializes superstructure subsystem */
  private Superstructure() {
    elevator = Elevator.getInstance();
    manipulator = Manipulator.getInstance();
    requestedPivotState = manipulator.getPivotState();

    mechanism = new SuperstructureMechanism(elevator::getPosMeters, manipulator::getPosRotations);
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
  }

  public boolean atTargetState() {
    return 
      manipulator.atTargetPivotState() &&
      manipulator.atTargetIntakeState() &&
      elevator.atTargetState();
  }

  public Command pivotTo(PivotState targetState) {
    return Commands.runOnce(() -> {
      if (elevator.getState() == ElevatorState.STOW) {
        requestedPivotState = targetState;
        manipulator.setTargetPivotState(targetState);
      }
    });
  };

  public Command elevatorTo(ElevatorState targetState) {
    return Commands
      .runOnce(() -> {
        if (manipulator.getPivotState().isUnsafe()) {
          manipulator.setTargetPivotState(PivotState.SAFE);
        }
      })
      .andThen(Commands.waitUntil(manipulator::atTargetPivotState))
      .andThen(() -> {
        elevator.setTargetState(ElevatorState.STOW);
      })
      .andThen(Commands.waitUntil(elevator::atTargetState))
      .andThen(() -> {
        elevator.setTargetState(targetState);
      })
      .andThen(Commands.waitUntil(elevator::atTargetState))
      .andThen(() -> {
        if (manipulator.getPivotState() != requestedPivotState) {
          manipulator.setTargetPivotState(requestedPivotState);
        }
      });
  }
}
