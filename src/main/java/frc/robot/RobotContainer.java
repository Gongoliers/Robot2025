// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    private final MotorValues values = new MotorValues();

    private final MotorOutputSim sim;


  /** Initializes the robot container */
  private RobotContainer() {
    driverController = new CommandXboxController(0);
    operatorController = new CommandXboxController(1);

    elevator = Elevator.getInstance();

    pivot = Pivot.getInstance();

      values.position.mut_replace(0.5, Rotations);
      var motor = new DCMotorSim(LinearSystemId.identifyPositionSystem(kV, kA), gearbox);
      sim = new MotorOutputSim(motor, Volts.zero(), () -> Volts.of(Math.cos(values.position.in(Radians)) * kG));

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
      return Commands.run(() -> {
          // Use feedback control to rest the arm at 0 radians
          double error = 0 - values.position.in(Radians);
          double fb = 0.1 * error;
          double ff = Math.cos(values.position.in(Radians)) * kG;
          sim.setVoltage(Volts.of(ff + fb));
          sim.updateValues(values, Seconds.of(0.02));
          SmartDashboard.putNumber("Simulated Position", values.position.in(Radians));
          SmartDashboard.putNumber("Simulated Speed", values.velocity.in(RadiansPerSecond));
          SmartDashboard.putNumber("Motor Voltage", values.motorVoltage.in(Volts));
    });
  }
}
