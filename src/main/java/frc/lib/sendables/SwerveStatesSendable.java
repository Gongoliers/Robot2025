package frc.lib.sendables;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.lib.controllers.swerve.SwerveModule;
import frc.robot.odometry.Odometry;

/** Nice swerve drive states sendable that allows for pretty visualization */
public class SwerveStatesSendable implements Sendable {

  private final SwerveModule nw;
  private final SwerveModule ne;
  private final SwerveModule sw;
  private final SwerveModule se;

  /**
   * Creates a swerve states sendable that can easily be used to visualize swerves in a dashboard
   * 
   * @param nwModule north-west swerve module
   * @param neModule north-east swerve module
   * @param swModule south-west swerve module
   * @param seModule south-east swerve module
   */
  public SwerveStatesSendable(
      SwerveModule nwModule,
      SwerveModule neModule,
      SwerveModule swModule,
      SwerveModule seModule) {
    
    nw = nwModule;
    ne = neModule;
    sw = swModule;
    se = seModule;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveDrive");

    builder.addDoubleProperty("Front Left Angle", () -> nw.getState().angle.getDegrees(), null);
    builder.addDoubleProperty("Front Left Velocity", () -> nw.getState().speedMetersPerSecond, null);

    builder.addDoubleProperty("Front Right Angle", () -> ne.getState().angle.getDegrees(), null);
    builder.addDoubleProperty("Front Right Velocity", () -> ne.getState().speedMetersPerSecond, null);

    builder.addDoubleProperty("Back Left Angle", () -> sw.getState().angle.getDegrees(), null);
    builder.addDoubleProperty("Back Left Velocity", () -> sw.getState().speedMetersPerSecond, null);

    builder.addDoubleProperty("Back Right Angle", () -> se.getState().angle.getDegrees(), null);
    builder.addDoubleProperty("Back Right Velocity", () -> se.getState().speedMetersPerSecond, null);

    builder.addDoubleProperty("Robot Angle", () -> Odometry.getInstance().getDriverRelativeHeading().getDegrees(), null);
  }
}
