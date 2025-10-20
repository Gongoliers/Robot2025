package frc.robot.elevator;

import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.controllers.position.PositionController;
import frc.lib.controllers.position.PositionControllerSim;
import frc.lib.controllers.position.PositionControllerTalonFXElevator;
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
  public static PositionController createElevatorPositionController(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(Subsystem.ELEVATOR)) {
      return new PositionControllerTalonFXElevator(
        config, 
        new CAN(10), 
        new CAN(11), 
        false);
    }

    return new PositionControllerSim();
  }
}
