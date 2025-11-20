package frc.lib.sensors;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.RobotConstants;
import java.util.function.Supplier;

/** Simulated gyroscope */
public class GyroscopeSim implements Gyroscope {

  private final Supplier<AngularVelocity> yawVelocity;

  private Angle yaw = Rotations.of(0.0);

  public GyroscopeSim(Supplier<AngularVelocity> yawVelocitySupplier) {
    this.yawVelocity = yawVelocitySupplier;
  }

  @Override
  public void configure() {}

  @Override
  public void getUpdatedVals(GyroscopeValues values) {
    values.yaw = yaw;
    values.yawVelociy = yawVelocity.get();
  }

  @Override
  public void setYaw(Angle newYaw) {
    this.yaw = newYaw;
  }

  @Override
  public void periodic() {
    yaw = yaw.plus(yawVelocity.get().times(Seconds.of(RobotConstants.PERIODIC_DURATION)));
  }
}
