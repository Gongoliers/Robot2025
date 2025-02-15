// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Telemetry;
import frc.robot.RobotConstants.Subsystem;
import frc.robot.auto.Auto;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorState;
import frc.robot.manipulator.IntakeState;
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

  /** Auto subsystem reference */
  private final Auto auto;

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
    auto = Auto.getInstance();

    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    Telemetry.initializeTabs(odometry, swerve, elevator, manipulator, superstructure, auto);

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

    operatorController.a().onTrue(superstructure.safelyTo(SuperstructureState.STOW));
    operatorController.b().onTrue(superstructure.safelyTo(SuperstructureState.L1));
    operatorController.y().onTrue(superstructure.safelyTo(SuperstructureState.L2));
    operatorController.x().onTrue(superstructure.safelyTo(SuperstructureState.L3));

    operatorController.povDown().onTrue(Commands.runOnce(() -> manipulator.setTargetIntakeState(IntakeState.CORALIN)));
    operatorController.povUp().onTrue(Commands.runOnce(() -> manipulator.setTargetIntakeState(IntakeState.CORALSCORIN)));
    operatorController.povLeft().onTrue(Commands.runOnce(() -> manipulator.setTargetIntakeState(IntakeState.STOP)));
  }

  public Command getAutonomousCommand() {
    if (RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.AUTO)) {
      return auto.getSelectedCommand();
    }

    return Commands.print("Auto disabled");
  }
}
