package frc.robot.endgame;

import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.controllers.position.EndgamePositionController;
import frc.lib.controllers.position.EndgamePositionControllerSim;
import frc.lib.controllers.position.EndgamePositionControllerTalonFX2;
import frc.lib.controllers.position.PositionController;
import frc.lib.controllers.position.PositionControllerSim;
import frc.lib.controllers.position.PositionControllerTalonFXPivot;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Creates pivot hardware */
public class EndgameFactory {
  
  /** 
   * Creates a pivot position controller motor 
   * 
   * @param config pivot mechanism config
   * @return a pivot position controller motor
   */
  public static EndgamePositionController createEndgameMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(Subsystem.ENDGAME)) {
      return new EndgamePositionControllerTalonFX2(
        new CAN(43),
        new CAN(44),
        config, 
        config.motorConfig().ccwPositive(), //TODO: this is dumb just pass config
        false);
    }

    return new EndgamePositionControllerSim();
  }
}
