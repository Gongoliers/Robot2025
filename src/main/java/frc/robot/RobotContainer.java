// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.CAN;
import frc.lib.Telemetry;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.controllers.position.EndgamePositionController;
import frc.lib.controllers.position.EndgamePositionControllerTalonFX2;
import frc.lib.targetting.ReefTarget;
import frc.robot.auto.Auto;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorState;
import frc.robot.endgame.Endgame;
import frc.robot.endgame.EndgameState;
import frc.robot.intake.Intake;
import frc.robot.intake.IntakeState;
import frc.robot.odometry.Odometry;
import frc.robot.pivot.Pivot;
import frc.robot.pivot.PivotState;
import frc.robot.ramp.Ramp;
import frc.robot.ramp.RampState;
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

  /** Pivot subsystem reference */
  private final Pivot pivot;

  /** Engame subsystem reference */
  private final Endgame endgame;

  /** Intake subsystem reference */
  private final Intake intake;

  /** Ramp subsystem reference */
  private final Ramp ramp;

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
    endgame = Endgame.getInstance();
    intake = Intake.getInstance();
    ramp = Ramp.getInstance();
    superstructure = Superstructure.getInstance();
    auto = Auto.getInstance();

    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    Telemetry.initializeTabs(odometry, swerve, elevator, pivot, endgame, intake, superstructure, auto, ramp);

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

    // driverController.povLeft().onTrue(auto.pathfindToTarget(ReefTarget.LEFT,
    // 0.05).andThen(Commands.print("worked")));
    // driverController.povUp().onTrue(auto.pathfindToTarget(ReefTarget.CENTER,
    // 0.05).andThen(Commands.print("worked")));
    // driverController.povRight().onTrue(auto.pathfindToTarget(ReefTarget.RIGHT,
    // 0.05).andThen(Commands.print("worked")));
    driverController.povUp().whileTrue(auto.forward()).onFalse(auto.stop());
    driverController.povDown().whileTrue(auto.backUp()).onFalse(auto.stop());
    driverController.rightBumper().whileTrue(auto.right()).onFalse(auto.stop());
    driverController.leftBumper().whileTrue(auto.left()).onFalse(auto.stop());

    driverController.leftStick().onTrue(Commands.runOnce(() -> endgame.setTargetState(EndgameState.STOW)));
    driverController.rightStick().onTrue(Commands.runOnce(() -> endgame.setTargetState(EndgameState.ARMED)));
    driverController.a().onTrue(endgame.climb());
    driverController.b().onTrue(endgame.stopClimb());

    operatorController.povLeft().onTrue(Commands.runOnce(() -> {
      intake.setTargetState(IntakeState.CORALINSLOW);
      ramp.setTargetState(RampState.INTAKESLOW);
    }));
    operatorController.povUp().onTrue(Commands.runOnce(() -> {
      intake.setTargetState(IntakeState.CORALINFAST);
      ramp.setTargetState(RampState.INTAKEFAST);
    }));
    operatorController.povRight().onTrue(Commands.runOnce(() -> {
      intake.setTargetState(IntakeState.STOP);
      ramp.setTargetState(RampState.STOP);
    }));

    operatorController.a().onTrue(superstructure.superstructureTo(SuperstructureState.STOW));
    operatorController.b().onTrue(superstructure.superstructureTo(SuperstructureState.L1));
    operatorController.x().onTrue(superstructure.superstructureTo(SuperstructureState.L2));
    operatorController.y().onTrue(superstructure.superstructureTo(SuperstructureState.L3));
    operatorController.rightBumper().onTrue(superstructure.superstructureTo(SuperstructureState.L4)
        .andThen(new RunCommand(() -> intake.setTargetState(IntakeState.CORALOUT), superstructure)).withTimeout(1.15)
        .andThen(Commands.runOnce(() -> intake.setTargetState(IntakeState.STOP), superstructure)));

    operatorController.leftBumper().onTrue(superstructure.intakeCoral());
  }

  public Command getAutonomousCommand() {
    if (RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.AUTO)) {
      return auto.getSelectedCommand();
    }

    return Commands.print("Auto disabled");
  }
}
