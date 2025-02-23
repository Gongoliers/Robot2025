package frc.lib.targetting;

import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Robot;

/** Gets field target positions with compensation for current alliance */
public class FieldTargetSupplier {
  
  public static Translation2d getReefCenter() {
    return (Robot.isRedAlliance())
      ? new Translation2d(13.0, 4.0)
      : new Translation2d(4.5, 4.0);
  }
}
