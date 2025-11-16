package frc.lib.sensors;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.CAN;
import frc.lib.configs.appliers.Pigeon2ConfigApplier;

/** Pigeon 2 gyroscope */
public class GyroscopePigeon2 implements Gyroscope {

  private Pigeon2 gyroscope;

  private final StatusSignal<Angle> roll, pitch, yaw;
  private final StatusSignal<AngularVelocity> rollVelocity, pitchVelocity, yawVelocity;

  public GyroscopePigeon2(CAN gyroscopeCAN) {

    gyroscope = new Pigeon2(gyroscopeCAN.id(), gyroscopeCAN.bus());

    roll = gyroscope.getRoll();
    pitch = gyroscope.getPitch();
    yaw = gyroscope.getYaw();

    rollVelocity = gyroscope.getAngularVelocityXWorld();
    pitchVelocity = gyroscope.getAngularVelocityYWorld();
    yawVelocity = gyroscope.getAngularVelocityZWorld();
  }

  @Override
  public void configure() {
    BaseStatusSignal.setUpdateFrequencyForAll(100, yaw, yawVelocity);
    BaseStatusSignal.setUpdateFrequencyForAll(10, roll, pitch, rollVelocity, pitchVelocity);

    Pigeon2ConfigApplier.applyFactoryDefault(gyroscope);
  }

  @Override
  public void getUpdatedVals(GyroscopeValues values) {
    BaseStatusSignal.refreshAll(roll, pitch, yaw, rollVelocity, pitchVelocity, yawVelocity);

    values.roll = Degrees.of(roll.getValueAsDouble());
    values.pitch = Degrees.of(pitch.getValueAsDouble());
    values.yaw = Degrees.of(yaw.getValueAsDouble());
    values.rollVelocity = DegreesPerSecond.of(rollVelocity.getValueAsDouble());
    values.pitchVelocity = DegreesPerSecond.of(pitchVelocity.getValueAsDouble());
    values.yawVelociy = DegreesPerSecond.of(yawVelocity.getValueAsDouble());
  }

  @Override
  public void setYaw(Angle newYaw) {
    gyroscope.setYaw(newYaw.in(Degrees));
  }

  @Override
  public void periodic() {}
}
