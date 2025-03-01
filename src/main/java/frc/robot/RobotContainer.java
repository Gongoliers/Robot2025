// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Telemetry;
import frc.lib.targetting.ReefTarget;
import frc.robot.auto.Auto;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorState;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.odometry.Odometry;
import frc.robot.pivot.Pivot;
import frc.robot.pivot.PivotState;
import frc.robot.superstructure.Superstructure;
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

  /** Pivot subsystem reference */
  private final Pivot pivot;

  /** Intake subsystem reference */
  private final Intake intake;

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
    pivot = Pivot.getInstance();
    intake = Intake.getInstance();
    superstructure = Superstructure.getInstance();
    auto = Auto.getInstance();

    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    Telemetry.initializeTabs(odometry, swerve, elevator, pivot, intake, superstructure, auto);

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
    swerve.setDefaultCommand(swerve.teleopDrive(driverController));
  }

  /** Configures controller bindings */
  private void configureBindings() {
    driverController.y().onTrue(odometry.setYaw(0.0));

    driverController.povLeft().onTrue(auto.pathfindToTarget(ReefTarget.LEFT, 0.05).andThen(Commands.print("worked")));
    driverController.povUp().onTrue(auto.pathfindToTarget(ReefTarget.CENTER, 0.05).andThen(Commands.print("worked")));
    driverController.povRight().onTrue(auto.pathfindToTarget(ReefTarget.RIGHT, 0.05).andThen(Commands.print("worked")));

    driverController.a().onTrue(odometry.trustVisionMeasurement("limelight-north"));

    operatorController.leftBumper().onTrue(pivot.zero());
    operatorController.rightBumper().onTrue(elevator.zero());    

    operatorController.a().onTrue(superstructure.elevatorTo(ElevatorState.STOW));
    operatorController.b().onTrue(superstructure.elevatorTo(ElevatorState.L2));
    operatorController.x().onTrue(superstructure.elevatorTo(ElevatorState.L3));

    operatorController.y().onTrue(superstructure.pivotTo(PivotState.TEST));
    operatorController.leftTrigger().onTrue(superstructure.pivotTo(PivotState.STOW));
    operatorController.rightTrigger().onTrue(superstructure.pivotTo(PivotState.TEST));

    operatorController.povDown().onTrue(Commands.runOnce(() -> intake.setTargetState(IntakeState.CORALIN)));
    operatorController.povUp().onTrue(Commands.runOnce(() -> intake.setTargetState(IntakeState.CORALOUT)));
    operatorController.povLeft().onTrue(Commands.runOnce(() -> intake.setTargetState(IntakeState.STOP)));
  }

  public Command getAutonomousCommand() {
    if (RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.AUTO)) {
      return auto.getSelectedCommand();
    }

    return Commands.print("Auto disabled");
  }
}
