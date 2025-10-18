// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Telemetry;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorState;

/** Robot container */
public class RobotContainer {

  /** Robot container singleton */
  private static RobotContainer instance = null;

  /** Driver controller */
  private final CommandXboxController driverController;

  /** Operator controller */
  private final CommandXboxController operatorController;
    ;

  /** Elevator subsystem reference */
  private final Elevator elevator;

  /** Initializes the robot container */
  private RobotContainer() {
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    elevator = Elevator.getInstance();

    Telemetry.initializeTabs(elevator);

    configureDefaultCommands();
    configureBindings();
  }

  /**
   * Returns the robot container
   * 
   * @return the robot container
   */
  public static RobotContainer getInstance() {
    if (instance == null) {
      instance = new RobotContainer();
    }

    return instance;
  }

  /** Configures subsystem default commands for teleop */
  public void configureDefaultCommands() {
    
  }

  /** Configures controller bindings */
  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
      return Commands.sequence(elevator.setTargetState(ElevatorState.L4), Commands.waitSeconds(3), elevator.setTargetState(ElevatorState.L1), Commands.waitSeconds(3)).repeatedly();
  }
}
