// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Telemetry;
import frc.robot.drive.Drive;
import frc.robot.drive.DriveFactory;
import frc.robot.elevator.Elevator;
import frc.robot.pivot.Pivot;

import static edu.wpi.first.units.Units.*;

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

  private final Drive drive;

  /** Initializes the robot container */
  private RobotContainer() {
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    elevator = Elevator.getInstance();

    pivot = Pivot.getInstance();

    drive = new Drive(DriveFactory.createSwerve());

    Telemetry.initializeTabs(drive);

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

  private ChassisSpeeds getFieldSpeeds() {
      LinearVelocity MAX_VELOCITY = MetersPerSecond.of(4);
      AngularVelocity MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(0.5);
      var x = MathUtil.applyDeadband(-driverController.getLeftY(), 0.1);
      var y = MathUtil.applyDeadband(-driverController.getLeftX(), 0.1);
      var omega = MathUtil.applyDeadband(-driverController.getRightX(), 0.1);
      return new ChassisSpeeds(
          MAX_VELOCITY.times(x),
          MAX_VELOCITY.times(y),
          MAX_ANGULAR_VELOCITY.times(omega)
      );
  }

  /** Configures subsystem default commands for teleop */
  public void configureDefaultCommands() {
      drive.setDefaultCommand(drive.drive(this::getFieldSpeeds));
  }

  /** Configures controller bindings */
  private void configureBindings() {
      // NOTE I deleted the operator binds for testing
      driverController.rightTrigger().whileTrue(drive.driveToward(this::getFieldSpeeds, drive::getNearestScoringPose));
  }

  public Command getAutonomousCommand() {
      return Commands.print("No autonomous command selected...");
  }
}
