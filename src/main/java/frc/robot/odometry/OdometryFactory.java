package frc.robot.odometry;

import frc.lib.sensors.GyroscopePigeon2;
import frc.lib.CAN;
import frc.lib.sensors.Gyroscope;
import frc.robot.Robot;

public class OdometryFactory {
  
  public static Gyroscope createGyroscope(Odometry odometry, CAN gyroscopeCAN) {
    if (Robot.isReal()) {
      return new GyroscopePigeon2(gyroscopeCAN);
    }

    //TODO: simulated gyroscope
    return new GyroscopePigeon2(gyroscopeCAN);
  }
}
