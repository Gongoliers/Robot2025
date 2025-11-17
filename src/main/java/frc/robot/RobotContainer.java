// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
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

  public Translation2d mixedVelocity(Translation2d driverVelocity, Pose2d current, Pose2d target) {
    final double KP = 4;
    final Distance MAX_DISTANCE = Meters.of(1);
    final Distance MIN_DISTANCE = Feet.of(1);

    Translation2d error = new Translation2d(target.getX() - current.getX(), target.getY() - current.getY());
    Rotation2d direction = error.getAngle();
    double distance = error.getNorm();
    SmartDashboard.putNumber("Distance (m)", distance);

    if (distance > MAX_DISTANCE.in(Meters)) {
        return driverVelocity;
    }

    double velocity = KP * distance;
    double x = velocity * direction.getCos();
    double y = velocity * direction.getSin();
    Translation2d proportionalVelocity = new Translation2d(x, y);

    if (distance < MIN_DISTANCE.in(Meters)) {
        return proportionalVelocity;
    }

    double t = distance / MAX_DISTANCE.in(Meters);
    SmartDashboard.putNumber("t", t);
    Translation2d scaledDriverVelocity = driverVelocity.times(t);
    Translation2d scaledProportionalVelocity = proportionalVelocity.times(1 - t);

    return scaledDriverVelocity.plus(scaledProportionalVelocity);
  }

  public Command getAutonomousCommand() {
    IdealSwerveSim swerve = new IdealSwerveSim();
    SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle();
    Field2d field = new Field2d();
    SmartDashboard.putData(field);
    Pose2d target = new Pose2d(Inches.of(144).in(Meters), Inches.of(153.5).in(Meters), Rotation2d.kZero);
    Translation2d driverVelocity = new Translation2d(MetersPerSecond.of(3).in(MetersPerSecond), MetersPerSecond.of(3).in(MetersPerSecond));

    return Commands.run(() -> {
        var pose = swerve.getState().Pose;
        field.setRobotPose(pose);
        var velocity = mixedVelocity(driverVelocity, pose, target);
        swerve.setControl(request.withVelocityX(velocity.getX()).withVelocityY(velocity.getY()));
    });
  }
}
