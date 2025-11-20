package frc.robot.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.motors.LossyMotorOutputSim;
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

    var motor =
        new MotorOutputSim(
            Volts.per(RotationsPerSecond).ofNative(config.feedforwardControllerConfig().kV()),
            Volts.per(RotationsPerSecondPerSecond)
                .ofNative(config.feedforwardControllerConfig().kA()),
            DCMotor.getKrakenX60(2));

    return new LossyMotorOutputSim(
        motor,
        Volts.of(config.feedforwardControllerConfig().kS()),
        (motorPosition) -> {
          return (motorPosition.in(Rotations) > 0)
              ? Volts.of(config.feedforwardControllerConfig().kG())
              : Volts.of(0.0);
        });
  }
}
