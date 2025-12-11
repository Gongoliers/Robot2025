package frc.robot.drive;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.PerUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.lib.NTDouble;

public class DriverAssistance {

  final NTDouble<PerUnit<LinearVelocityUnit, DistanceUnit>> GAIN;
  final NTDouble<DistanceUnit> MIN_DISTANCE;
  final NTDouble<DistanceUnit> MAX_DISTANCE;

  public DriverAssistance(
      Per<LinearVelocityUnit, DistanceUnit> gain, Distance minDistance, Distance maxDistance) {
    GAIN = new NTDouble<>("DriverAssistance.Gain", MetersPerSecond.per(Meter), gain);
    MIN_DISTANCE = new NTDouble<>("DriverAssistance.MinDistance", Meters, minDistance);
    MAX_DISTANCE = new NTDouble<>("DriverAssistance.MaxDistance", Meters, maxDistance);
  }

  public boolean hasDeadSpots(LinearVelocity maxSpeed) {
    // TODO Document this formula
    return MIN_DISTANCE.get().timesConversionFactor(GAIN.get()).lte(maxSpeed.times(2));
  }

  private Translation2d error(Pose2d pose, Pose2d target) {
    return target.getTranslation().minus(pose.getTranslation());
  }

  private Distance errorMagnitude(Pose2d pose, Pose2d target) {
    return Meters.of(error(pose, target).getNorm());
  }

  private Rotation2d errorDirection(Pose2d pose, Pose2d target) {
    return error(pose, target).getAngle();
  }

  private Pose2d poseAlongLine(Pose2d target, Rotation2d direction, Distance distance) {
    Translation2d offset = new Translation2d(distance.in(Meters), direction);
    Translation2d alongLine = target.getTranslation().minus(offset);
    return new Pose2d(alongLine, direction);
  }

  private ChassisSpeeds createChassisSpeeds(LinearVelocity velocity, Rotation2d direction) {
    return new ChassisSpeeds(
        velocity.times(direction.getCos()),
        velocity.times(direction.getSin()),
        RotationsPerSecond.zero());
  }

  private Translation2d fieldVelocity(ChassisSpeeds fieldSpeeds) {
    return new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
  }

  private LinearVelocity fieldVelocityMagnitude(ChassisSpeeds fieldSpeeds) {
    return MetersPerSecond.of(fieldVelocity(fieldSpeeds).getNorm());
  }

  private ChassisSpeeds clampedSum(ChassisSpeeds controlSpeeds, ChassisSpeeds otherSpeeds) {
    ChassisSpeeds combinedSpeeds = controlSpeeds.plus(otherSpeeds);
    LinearVelocity combinedVelocity = fieldVelocityMagnitude(combinedSpeeds);
    LinearVelocity controlVelocity = fieldVelocityMagnitude(controlSpeeds);
    LinearVelocity clampedVelocity =
        (LinearVelocity) Measure.min(controlVelocity, combinedVelocity);
    return createChassisSpeeds(clampedVelocity, fieldVelocity(combinedSpeeds).getAngle());
  }

  public void drawDebugObjects(Field2d field, Pose2d pose, Pose2d target) {
    field.getObject("DriverAssistance.TargetPose").setPose(target);

    Rotation2d direction = errorDirection(pose, target);

    field
        .getObject("DriverAssistance.MinPose")
        .setPose(poseAlongLine(target, direction, (Distance) MIN_DISTANCE.get()));
    field
        .getObject("DriverAssistance.MaxPose")
        .setPose(poseAlongLine(target, direction, (Distance) MAX_DISTANCE.get()));
  }

  public ChassisSpeeds applyDriverAssistance(
      ChassisSpeeds fieldSpeeds, Pose2d pose, Pose2d target) {
    Distance distance = errorMagnitude(pose, target);
    Rotation2d direction = errorDirection(pose, target);

    if (distance.gt(MAX_DISTANCE.get())) {
      return fieldSpeeds;
    }

    LinearVelocity assistAmount = distance.timesConversionFactor(GAIN.get());
    ChassisSpeeds assistSpeeds = createChassisSpeeds(assistAmount, direction);

    if (distance.lt(MIN_DISTANCE.get())) {
      return assistSpeeds;
    }

    return clampedSum(fieldSpeeds, assistSpeeds);
  }
}
