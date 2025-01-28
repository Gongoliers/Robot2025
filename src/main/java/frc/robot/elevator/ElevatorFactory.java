package frc.robot.elevator;

import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.controllers.position.LinearPositionController;
import frc.lib.controllers.position.LinearPositionControllerElevator;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Creates elevator hardware */
public class ElevatorFactory {
  
  public static LinearPositionController createDriveMotor(
      CAN leaderCAN,
      CAN followerCAN,
      MechanismConfig config) {

    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(Subsystem.ELEVATOR)) {
      return new LinearPositionControllerElevator(
        leaderCAN,
        followerCAN,
        config,
        false,
        false);
    }

    return new LinearPositionControllerElevator(
      leaderCAN, 
      followerCAN, 
      config, 
      false, 
      false);
  }
}
