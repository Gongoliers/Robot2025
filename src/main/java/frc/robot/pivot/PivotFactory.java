package frc.robot.pivot;

import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.controllers.position.PositionController;
import frc.lib.controllers.position.PositionControllerSim;
import frc.lib.controllers.position.PositionControllerTalonFXPivot;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

public class PivotFactory {
  
  public static PositionController createPivotMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(Subsystem.PIVOT)) {
      return new PositionControllerTalonFXPivot(
        new CAN(20), 
        config, 
        config.motorConfig().ccwPositive(), //TODO: this is dumb just pass config
        false);
    }

    return new PositionControllerSim();
  }
}
