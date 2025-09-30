// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
  public RobotContainer() {
    swerve = new Swerve();
    odometry = new Odometry(swerve.createPoseEstimator(Rotation2d.kZero, new Pose2d()));

    swerve.setDriverRotationSupplier(odometry::getDriverRelativeHeading);
    odometry.setChassisSpeedsSupplier(swerve::getChassisSpeeds);
    odometry.setModulePositionsSupplier(swerve::getModulePositions);

    elevator = new Elevator();
    pivot = new Pivot();
    endgame = new Endgame();
    intake = new Intake();
    ramp = new Ramp();
    superstructure = new Superstructure(elevator, pivot, intake, ramp, endgame);
    auto = new Auto(odometry, swerve, superstructure);

    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    Telemetry.initializeTabs(odometry, swerve, elevator, pivot, endgame, intake, superstructure, auto, ramp);

    configureDefaultCommands();
    configureBindings();
  }

  /** Configures subsystem default commands for teleop */
  public void configureDefaultCommands() {
    swerve.setDefaultCommand(swerve.teleopDrive(driverController));
  }

  /** Configures controller bindings */
  private void configureBindings() {
    driverController.y().onTrue(odometry.setYaw(0.0));

    driverController.povLeft().onTrue(auto.allign(ReefTarget.LEFT, 0.2));
    driverController.povRight().onTrue(auto.allign(ReefTarget.RIGHT, 0.2));
    
    driverController.povUp().whileTrue(auto.forward()).onFalse(auto.stop());
    driverController.povDown().whileTrue(auto.backUp()).onFalse(auto.stop());
    driverController.rightBumper().whileTrue(auto.right()).onFalse(auto.stop());
    driverController.leftBumper().whileTrue(auto.left()).onFalse(auto.stop());

    driverController.x().onTrue(elevator.zero());

    //for testing
    operatorController.a().onTrue(odometry.trustVisionMeasurement("limelight-west"));

    operatorController.leftStick().onTrue(Commands.runOnce(() -> endgame.setTargetState(EndgameState.HOOK)));
    operatorController.povDown().onTrue(superstructure.manualClimb(operatorController));

    operatorController.povLeft().onTrue(superstructure.scoreCoral());
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
    operatorController.rightBumper().onTrue(superstructure.superstructureTo(SuperstructureState.L4));

    operatorController.leftBumper().onTrue(superstructure.intakeCoral());

    operatorController.rightStick().onTrue(superstructure.cancelAll());
  }

  public Command getAutonomousCommand() {
    if (RobotConstants.ENABLED_SUBSYSTEMS.contains(RobotConstants.Subsystem.AUTO)) {
      return auto.getSelectedCommand();
    }

    return Commands.print("Auto disabled");
  }
}
