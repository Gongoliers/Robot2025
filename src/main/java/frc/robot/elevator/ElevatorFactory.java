package frc.robot.elevator;

import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.controllers.position.ElevatorPositionController;
import frc.lib.controllers.position.ElevatorPositionControllerElevator;
import frc.lib.controllers.position.ElevatorPositionControllerSim;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Creates elevator hardware */
public class ElevatorFactory {
  
  public static ElevatorPositionController createDriveMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(Subsystem.ELEVATOR)) {
      return new ElevatorPositionControllerElevator(
        new CAN(10),
        new CAN(11),
        config,
        false,
        false,
        1/26.518545);
    }

    return new ElevatorPositionControllerSim();
  }
}
