package frc.robot.intake;

import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.controllers.velocity.VelocityController;
import frc.lib.controllers.velocity.VelocityControllerSim;
import frc.lib.controllers.velocity.VelocityControllerTalonFXPIDF;
import frc.lib.sensors.TimeOfFlight;
import frc.lib.sensors.TimeOfFlightCANrange;
import frc.lib.sensors.TimeOfFlightSim;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

/** Creates intake hardware */
public class IntakeFactory {
  
  public static VelocityController createIntakeMotor(MechanismConfig config) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(Subsystem.INTAKE)) {
      return new VelocityControllerTalonFXPIDF(
        new CAN(30), 
        config, 
        false);
    }

    return new VelocityControllerSim();
  }

  public static TimeOfFlight createTimeOfFlightSensor() {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(Subsystem.INTAKE)) {
      return new TimeOfFlightCANrange(
        new CAN(40));
    }

    return new TimeOfFlightSim();
  }
}
