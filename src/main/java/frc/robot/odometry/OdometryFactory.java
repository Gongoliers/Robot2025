package frc.robot.odometry;

import frc.lib.sensors.GyroscopePigeon2;
import frc.lib.sensors.GyroscopeSim;
import edu.wpi.first.math.util.Units;
import frc.lib.CAN;
import frc.lib.sensors.Gyroscope;
import frc.robot.Robot;

public class OdometryFactory {
  
  /**
   * Creates the gyroscope
   * 
   * @param odometry odometry instance
   * @param gyroscopeCAN CAN id and bus of gyroscope
   * @return the gyroscope
   */
  public static Gyroscope createGyroscope(Odometry odometry, CAN gyroscopeCAN) {
    if (Robot.isReal()) {
      return new GyroscopePigeon2(gyroscopeCAN);
    }

    return new GyroscopeSim(() -> Units.radiansToRotations(odometry.getVelocity().dtheta));
  }
}
