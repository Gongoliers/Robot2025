package frc.robot.odometry;

import frc.lib.sensors.GyroscopePigeon2;
import frc.lib.sensors.GyroscopeSim;
import frc.lib.targetting.Limelights;
import frc.lib.targetting.Limelights3G;
import frc.lib.targetting.LimelightsSim;
import edu.wpi.first.math.util.Units;
import frc.lib.CAN;
import frc.lib.sensors.Gyroscope;
import frc.robot.Robot;
import frc.robot.RobotConstants;
import frc.robot.RobotConstants.Subsystem;

public class OdometryFactory {
  
  /**
   * Creates the gyroscope
   * 
   * @param odometry odometry instance
   * @return the gyroscope
   */
  public static Gyroscope createGyroscope(Odometry odometry) {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(Subsystem.ODOMETRY)) {
      return new GyroscopePigeon2(new CAN(0));
    }

    return new GyroscopeSim(() -> Units.radiansToRotations(odometry.getVelocity().dtheta));
  }

  /**
   * Creates limelights
   * 
   * @return limelights class
   */
  public static Limelights createLimelights() {
    if (Robot.isReal() && RobotConstants.ENABLED_SUBSYSTEMS.contains(Subsystem.LIMELIGHT) && RobotConstants.ENABLED_SUBSYSTEMS.contains(Subsystem.ODOMETRY)) {
      return new Limelights3G();
    }

    return new LimelightsSim();
  }
}
