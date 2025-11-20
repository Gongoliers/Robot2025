// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Telemetry;
import frc.robot.drive.Drive;
import frc.robot.drive.DriveFactory;

import static edu.wpi.first.units.Units.Meters;

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
  // private final Elevator elevator;

  /** Pivot subsystem reference */
  // private final Pivot pivot;

  private final Drive drive;

  /** Initializes the robot container */
  private RobotContainer() {
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    // elevator = Elevator.getInstance();

    // pivot = Pivot.getInstance();

    drive = new Drive(DriveFactory.createSwerve());
    drive.setDefaultCommand(getAutonomousCommand());

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

  /** Configures subsystem default commands for teleop */
  public void configureDefaultCommands() {
    
  }

  /** Configures controller bindings */
  private void configureBindings() {
  }

  public Translation2d mixedVelocity(Translation2d driverVelocity, Pose2d current, Pose2d target) {
    final double KP = 2;
    final Distance MAX_DISTANCE = Meters.of(2);
    final Distance MIN_DISTANCE = Meters.of(0.5);

    Translation2d error = new Translation2d(target.getX() - current.getX(), target.getY() - current.getY());
    Rotation2d direction = error.getAngle();
    double distance = error.getNorm();
    SmartDashboard.putNumber("Distance (m)", distance);

    if (distance > MAX_DISTANCE.in(Meters)) {
        return driverVelocity;
    }

    double velocity = Math.min(driverVelocity.getNorm(), KP * distance);
    Translation2d proportionalVelocity = new Translation2d(velocity, direction);

    if (distance < MIN_DISTANCE.in(Meters)) {
        return proportionalVelocity;
    }

    double t = distance / MAX_DISTANCE.in(Meters);
    Translation2d scaledDriverVelocity = driverVelocity.times(t);
    Translation2d scaledProportionalVelocity = proportionalVelocity.times(1 - t);

    return scaledDriverVelocity.plus(scaledProportionalVelocity);
  }

  public Command getAutonomousCommand() {
    Pose2d target = new Pose2d(Meters.of(3.286).in(Meters), Meters.of(1.34 ).in(Meters), Rotation2d.kZero);
      SmartDashboard.putNumber("Target X", Units.metersToFeet(target.getX()));
      SmartDashboard.putNumber("Target Y", Units.metersToFeet(target.getY()));

    return drive.driveFacing(() -> {
        var pose = drive.getPose();

        var x = MathUtil.applyDeadband(-driverController.getLeftY(), 0.1);
        var y = MathUtil.applyDeadband(-driverController.getLeftX(), 0.1);

        var velocity = new Translation2d(x, y);

        boolean slow = driverController.getLeftTriggerAxis() > 0.5;
        boolean assist = driverController.getRightTriggerAxis() > 0.5;

        SmartDashboard.putBoolean("Slow?", slow);
        SmartDashboard.putBoolean("Assist?", assist);

        if (slow) {
            velocity = velocity.times(0.5);
        }
        if (assist) {
            velocity = mixedVelocity(velocity, pose, target);
        }

        SmartDashboard.putNumber("Velocity", velocity.getNorm());

        return ChassisSpeeds.fromFieldRelativeSpeeds(velocity.getX(), velocity.getY(), 0, pose.getRotation());
    }, target::getRotation);
  }
}
