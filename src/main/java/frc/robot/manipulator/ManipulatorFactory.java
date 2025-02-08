package frc.robot.manipulator;

import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.controllers.position.PositionController;
import frc.lib.controllers.position.PositionControllerSim;
import frc.lib.controllers.position.PositionControllerTalonFXPivot;
import frc.lib.controllers.velocity.VelocityController;
import frc.lib.controllers.velocity.VelocityControllerSim;
import frc.lib.controllers.velocity.VelocityControllerTalonFXPIDF;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Creates manipulator hardware */
public class ManipulatorFactory {
  
  public static PositionController createPivotMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(Subsystem.MANIPULATOR)) {
      return new PositionControllerTalonFXPivot(
        new CAN(20), 
        config, 
        false, 
        false);
    }

    return new PositionControllerSim();
  }

  public static VelocityController createIntakeMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(Subsystem.MANIPULATOR)) {
      return new VelocityControllerTalonFXPIDF(
        new CAN(30), 
        config, 
        false);
    }

    return new VelocityControllerSim();
  }
}
