// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.Telemetry;
import frc.lib.motors.MotorOutputSim;
import frc.lib.motors.MotorValues;
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

    /**
     * System identification constants, for simulation.
     */
    private final double kS = 0.12;
    private final double kG = 0.575;
    private final double kV = 0.1;
    private final double kA = 0.09;
    private final DCMotor gearbox = DCMotor.getKrakenX60(2);

    private final MotorOutputSim sim;

    private final ElevatorFeedforward feedforward;

  /** Initializes the robot container */
  private RobotContainer() {
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    elevator = Elevator.getInstance();

    pivot = Pivot.getInstance();

      var motor = new DCMotorSim(LinearSystemId.identifyPositionSystem(kV, kA), gearbox);
      sim = new MotorOutputSim(motor, Volts.of(kS), Volts.of(kG));
      feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

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
    operatorController.y().onTrue(elevator.setPosition(Meters.of(0)));
  }

  public Command getAutonomousCommand() {
      var goalSpeed = RadiansPerSecond.of(10);
      MotorValues values = new MotorValues();

      return Commands.run(() -> {
          sim.setVoltage(Volts.of(feedforward.calculate(goalSpeed.in(RadiansPerSecond))));
          sim.updateValues(values, Seconds.of(0.02));
          SmartDashboard.putNumber("Simulated Position", values.position.in(Radians));
          SmartDashboard.putNumber("Simulated Speed", values.velocity.in(RadiansPerSecond));
    });
  }
}
