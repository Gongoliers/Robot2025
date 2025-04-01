package frc.robot.ramp;

import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.controllers.velocity.VelocityController;
import frc.lib.controllers.velocity.VelocityControllerSim;
import frc.lib.controllers.velocity.VelocityControllerTalonFXPIDF;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Creates ramp hardware */
public class RampFactory {
  
  public static VelocityController createRampMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(Subsystem.INTAKE)) {
      return new VelocityControllerTalonFXPIDF(
        new CAN(35), 
        config, 
        false);
    }

    return new VelocityControllerSim();
  }
}
