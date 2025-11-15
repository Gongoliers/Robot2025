package frc.robot.elevator;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.motors.MotorOutput;
import frc.lib.motors.MotorOutputSim;
import frc.lib.motors.MotorOutputTalonFX2;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Creates elevator hardware */
public class ElevatorFactory {

    /**
     * Creates an elevator position controller
     *
     * @return an elevator position controller
     */
    public static MotorOutput createMotorOutput(MechanismConfig config) {
        if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(Subsystem.ELEVATOR)) {
            return new MotorOutputTalonFX2(config.motorConfig(), new CAN(10), new CAN(11), false);
        }

        var plant =
                LinearSystemId.identifyPositionSystem(
                        config.feedforwardControllerConfig().kV(),
                        config.feedforwardControllerConfig().kA());
        var sim = new DCMotorSim(plant, DCMotor.getKrakenX60(2));
        return new MotorOutputSim(
                sim,
                Volts.of(config.feedforwardControllerConfig().kS()),
                Volts.of(config.feedforwardControllerConfig().kG()));
    }
}
