// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Telemetry;
import frc.lib.swerves.IdealSwerveSim;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorState;
import frc.robot.pivot.Pivot;

/** Robot container */
public class RobotContainer {

  /** Robot container singleton */
  private static RobotContainer instance = null;

  /** Driver controller */
  private final CommandXboxController driverController;

  /** Operator controller */
  private final CommandXboxController operatorController;

  /** Multithreader */
  private final Multithreader multithreader;

  /** Elevator subsystem reference */
  private final Elevator elevator;

  /** Pivot subsystem reference */
  private final Pivot pivot;

  /** Initializes the robot container */
  private RobotContainer() {
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    elevator = Elevator.getInstance();

    pivot = Pivot.getInstance();

    Telemetry.initializeTabs(elevator, pivot);

    multithreader = Multithreader.getInstance();
    multithreader.start();

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
    operatorController.a().onTrue(elevator.setTargetState(ElevatorState.STOW));
    operatorController.b().onTrue(elevator.setTargetState(ElevatorState.L1));
    operatorController.x().onTrue(elevator.setTargetState(ElevatorState.L2));
    operatorController.y().onTrue(elevator.setElevatorPosition(Meters.of(0)));
  }

  public Command getAutonomousCommand() {
    IdealSwerveSim swerve = new IdealSwerveSim();
    SwerveRequest request = new SwerveRequest.FieldCentricFacingAngle().withVelocityX(0.1).withTargetDirection(Rotation2d.k180deg);
    Field2d field = new Field2d();
    SmartDashboard.putData(field);

    return Commands.run(() -> {
        swerve.setControl(request);
        field.setRobotPose(swerve.getState().Pose);
    });
  }
}
