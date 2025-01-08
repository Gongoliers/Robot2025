package frc.lib.sensors;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.lib.CAN;
import frc.lib.configs.appliers.Pigeon2ConfigApplier;

/** Pigeon 2 gyroscope */
public class GyroscopePigeon2 implements Gyroscope {
  
  private Pigeon2 gyroscope;

  private final StatusSignal<Angle> roll, pitch, yaw;
  private final StatusSignal<AngularVelocity> rollVelocity, pitchVelocity, yawVelocity;

  public GyroscopePigeon2(
      CAN gyroscopeCAN) {
    
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
    values.rollRotations = Units.degreesToRotations(roll.getValueAsDouble());
    values.pitchRotations = Units.degreesToRotations(pitch.getValueAsDouble());
    values.yawRotations = Units.degreesToRotations(yaw.getValueAsDouble());
    values.rollVelRotationsPerSec = Units.degreesToRotations(rollVelocity.getValueAsDouble());
    values.pitchVelRotationsPerSec = Units.degreesToRotations(pitchVelocity.getValueAsDouble());
    values.yawVelRotationsPerSec = Units.degreesToRotations(yawVelocity.getValueAsDouble());
  }

  @Override
  public void setYaw(double yawRotations) {
    gyroscope.setYaw(Units.rotationsToDegrees(yawRotations));
  }

  @Override
  public void periodic() {}
}
