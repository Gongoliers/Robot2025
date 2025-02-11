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
import frc.robot.manipulator.Manipulator;
import frc.robot.manipulator.PivotState;
import frc.robot.odometry.Odometry;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureState;
import frc.robot.swerve.Swerve;

/** Robot container */
public class RobotContainer {

  /** Robot container singleton */
  private static RobotContainer instance = null;

  /** Odometry subsystem reference */
  private final Odometry odometry;

  /** Swerve subsystem reference */
  private final Swerve swerve;

  /** Elevator subsystem reference */
  private final Elevator elevator;

  /** Manipulator subsystem reference */
  private final Manipulator manipulator;

  /** Superstructure subystem reference */
  private final Superstructure superstructure;

  /** Driver controller */
  private final CommandXboxController driverController;

  /** Operator controller */
  private final CommandXboxController operatorController;

  /** Initializes the robot container */
  private RobotContainer() {
    odometry = Odometry.getInstance();
    swerve = Swerve.getInstance();
    elevator = Elevator.getInstance();
    manipulator = Manipulator.getInstance();
    superstructure = Superstructure.getInstance();

    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    Telemetry.initializeTabs(odometry, swerve, elevator, manipulator, superstructure);

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

  /** Configures subsystem default commands */
  private void configureDefaultCommands() {
    swerve.setDefaultCommand(swerve.teleopDrive(driverController));
  }

  /** Configures controller bindings */
  private void configureBindings() {
    driverController.y().onTrue(odometry.zeroYaw());

    operatorController.leftBumper().onTrue(manipulator.zeroPivot());
    operatorController.rightBumper().onTrue(elevator.zero());    

    operatorController.a().onTrue(Commands.runOnce(() -> elevator.setTargetState(ElevatorState.STOW)));
    operatorController.b().onTrue(Commands.runOnce(() -> elevator.setTargetState(ElevatorState.L1)));
    operatorController.y().onTrue(Commands.runOnce(() -> elevator.setTargetState(ElevatorState.L2)));

    operatorController.povDown().onTrue(Commands.runOnce(() -> manipulator.setTargetPivotState(PivotState.STOW)));
    operatorController.povUp().onTrue(Commands.runOnce(() -> manipulator.setTargetPivotState(PivotState.SAFE)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous commands configured");
  }
}
