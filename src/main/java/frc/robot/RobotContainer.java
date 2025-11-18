// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Telemetry;
import frc.robot.elevator.Elevator;
import frc.robot.elevator.ElevatorState;
import frc.robot.pivot.Pivot;
import frc.robot.roller.Roller;

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

    private final Roller roller;

    /** Initializes the robot container */
    private RobotContainer() {
        driverController = new CommandXboxController(0);
        operatorController = new CommandXboxController(1);

        elevator = Elevator.getInstance();

        pivot = Pivot.getInstance();

        roller = new Roller();

        Telemetry.initializeTabs(elevator, pivot, roller);

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
    public void configureDefaultCommands() {}

    /** Configures controller bindings */
    private void configureBindings() {
//        operatorController
//            .a()
//            .onTrue(elevator.setTargetState(ElevatorState.STOW));
//        operatorController
//            .b()
//            .onTrue(elevator.setTargetState(ElevatorState.L1));
//        operatorController
//            .x()
//            .onTrue(elevator.setTargetState(ElevatorState.L2));
//        operatorController
//            .y()
//            .onTrue(elevator.setElevatorPosition(Meters.of(0)));

        driverController
            .a()
            .whileTrue(
                roller.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
            );
        driverController
            .b()
            .whileTrue(
                roller.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
            );
        driverController
            .x()
            .whileTrue(roller.sysIdDynamic(SysIdRoutine.Direction.kForward));
        driverController
            .y()
            .whileTrue(roller.sysIdDynamic(SysIdRoutine.Direction.kReverse));

        driverController.leftTrigger().whileTrue(roller.runVoltage(Volts.of(-6)));
        driverController.rightTrigger().whileTrue(roller.runVoltage(Volts.of(6)));
    }

    public Command getAutonomousCommand() {
        return Commands.print("Auto disabled");
    }
}
