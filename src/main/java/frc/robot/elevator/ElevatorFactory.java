package frc.robot.elevator;

import static edu.wpi.first.units.Units.KilogramSquareMeters;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.controllers.position.PositionController;
import frc.lib.controllers.position.PositionControllerSim;
import frc.lib.controllers.position.PositionControllerTalonFXElevator;
import frc.lib.motors.MotorOutput;
import frc.lib.motors.MotorOutputFlywheelSim;
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
      return new MotorOutputTalonFX2(
        config.motorConfig(), 
        new CAN(10), 
        new CAN(11), 
        false);
    }

    //TODO: this is a temporary sim that is not at all usable
    return new MotorOutputFlywheelSim(
      config.motorConfig(), 
      new DCMotor(1, 1, 1, 1, 1, 1),
      KilogramSquareMeters.of(1.0),
      1.0);
  }
}
