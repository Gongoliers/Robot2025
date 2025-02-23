package frc.robot.auto;

import choreo.auto.AutoFactory;
import frc.lib.Subsystem;
import frc.robot.odometry.Odometry;
import frc.robot.swerve.Swerve;

/** Subsystem that handles all auto driving */
public class Auto extends Subsystem {
  
  /** Instance of auto subsystem */
  private static Auto instance = null;

  /** Choreo auto factory */
  private static AutoFactory autoFactory;

  public static Auto getInstance() {
    if (instance == null) {
      instance = new Auto();
    }

    return instance;
  }

  private Auto() {
    autoFactory = new AutoFactory(
      Odometry.getInstance()::getPosition, 
      Odometry.getInstance()::setPosition, 
      Swerve.getInstance()::autoDrive, 
      true, 
      Swerve.getInstance());
  }

  @Override
  public void initializeTab() {

  }

  @Override
  public void periodic() {
    
  }
}
