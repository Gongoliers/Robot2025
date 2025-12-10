package frc.robot.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.NTDouble;
import frc.lib.Subsystem;
import frc.lib.swerves.SwerveOutput;
import java.util.List;
import java.util.function.Predicate;
import java.util.function.Supplier;

public class Drive extends Subsystem {

  private final SwerveOutput swerve;

  private SwerveDrivetrain.SwerveDriveState state;

  private final Field2d field;

  private final AprilTagFieldLayout tagLayout;

  private final List<Pose2d> scoringPoses;

  private boolean hasSetPerspective = false;

  public Drive(SwerveOutput swerve) {
    this.swerve = swerve;
    this.state = new SwerveDrivetrain.SwerveDriveState();
    this.field = new Field2d();
    this.tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    Predicate<AprilTag> isBlueScoringTag = tag -> 17 <= tag.ID && tag.ID <= 22;
    Predicate<AprilTag> isRedScoringTag = tag -> 6 <= tag.ID && tag.ID <= 11;
    this.scoringPoses =
        tagLayout.getTags().stream()
            .filter(isBlueScoringTag.or(isRedScoringTag))
            .map(tag -> tag.pose.toPose2d())
            .map(pose -> pose.rotateAround(pose.getTranslation(), Rotation2d.k180deg))
            .toList();
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    tab.add(field);
    tab.addDoubleArray(
        "States",
        () -> {
          SwerveModuleState[] states = state.ModuleStates;
          double[] doubles = new double[8];

          if (states != null) {
            for (int i = 0; i < 4; i++) {
              SwerveModuleState state = states[i];
              doubles[2 * i] = state.angle.getDegrees();
              doubles[2 * i + 1] = state.speedMetersPerSecond;
            }
          }

          return doubles;
        });
    tab.addDoubleArray(
        "Targets",
        () -> {
          SwerveModuleState[] states = state.ModuleTargets;
          double[] doubles = new double[8];

          if (states != null) {
            for (int i = 0; i < 4; i++) {
              SwerveModuleState state = states[i];
              doubles[2 * i] = state.angle.getDegrees();
              doubles[2 * i + 1] = state.speedMetersPerSecond;
            }
          }

          return doubles;
        });
  }

  @Override
  public void periodic() {
    state = swerve.getState();
    field.setRobotPose(state.Pose);
    // field.getObject("target").setPose(getNearestScoringPose());

    // NOTE This was taken from the generated project, unsure if it is needed
    // trySettingPerspective();
  }

  private void trySettingPerspective() {
    if (!hasSetPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                swerve.setOperatorPerspectiveForward(
                    allianceColor == DriverStation.Alliance.Red
                        ? Rotation2d.k180deg
                        : Rotation2d.kZero);
                hasSetPerspective = true;
              });
    }
  }

  public Pose2d getPose() {
    return state.Pose;
  }

  public Pose2d getNearestScoringPose() {
    return getPose().nearest(scoringPoses);
  }

  public Field2d getField() {
    return field;
  }

  public SysIdRoutine createDriveRoutine() {
    return DriveFactory.createDriveRoutine(swerve, this);
  }

  public SysIdRoutine createSteerRoutine() {
    return DriveFactory.createSteerRoutine(swerve, this);
  }

  public SysIdRoutine createRotationRoutine() {
    return DriveFactory.createRotationRoutine(swerve, this);
  }

  public Command sysIdQuasistatic(SysIdRoutine routine, SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine routine, SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public Command drive(Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
    // TODO Make factory for requests
    SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

    return run(
        () -> {
          ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
          swerve.setControl(
              request
                  .withVelocityX(fieldSpeeds.vxMetersPerSecond)
                  .withVelocityY(fieldSpeeds.vyMetersPerSecond)
                  .withRotationalRate(fieldSpeeds.omegaRadiansPerSecond));
        });
  }

  public Command driveFacing(
      Supplier<ChassisSpeeds> fieldSpeedsSupplier, Supplier<Rotation2d> directionSupplier) {
    // TODO Make factory for requests
    SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle();

    final NTDouble KP = new NTDouble<>("driveFacing.KP", RotationsPerSecond.per(Rotation));
    final NTDouble MAX_ROTATIONAL_RATE = new NTDouble("driveFacing.MAX_ROTATIONAL_RATE", RotationsPerSecond);

    return run(
        () -> {
          ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
          Rotation2d direction = directionSupplier.get();
          swerve.setControl(
              request
                  .withVelocityX(fieldSpeeds.vxMetersPerSecond)
                  .withVelocityY(fieldSpeeds.vyMetersPerSecond)
                  .withTargetDirection(direction)
                  .withHeadingPID(KP.get().in(RadiansPerSecond.per(Radian)), 0, 0)
                  .withMaxAbsRotationalRate(MAX_ROTATIONAL_RATE.get().in(RadiansPerSecond)));
        });
  }

  public Command driveToward(
      Supplier<ChassisSpeeds> fieldSpeedsSupplier, Supplier<Pose2d> targetPoseSupplier) {

    final NTDouble GAIN = new NTDouble("driveToward.GAIN", MetersPerSecond.per(Meter), MetersPerSecond.per(Meter).ofNative(16));
    final NTDouble MIN_DISTANCE = new NTDouble("driveToward.MIN_DISTANCE", Meters, Meters.of(0.5));
    final NTDouble MAX_DISTANCE = new NTDouble("driveToward.MAX_DISTANCE", Meters, Meters.of(1));

    // TODO Make utility class with closures for mutations
    return driveFacing(
        () -> {
          Per<LinearVelocityUnit, DistanceUnit> gain  = (Per<LinearVelocityUnit, DistanceUnit>) GAIN.get();
          Distance minDistance = (Distance) MIN_DISTANCE.get();
          Distance maxDistance = (Distance) MAX_DISTANCE.get();

          SmartDashboard.putBoolean("No Dead Spots?", maxDistance.timesConversionFactor(gain).gt(TunerConstants.kSpeedAt12Volts.times(2)));

          ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
          Translation2d fieldVelocity =
              new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
          Pose2d pose = getPose();
          Pose2d targetPose = targetPoseSupplier.get();
          field.getObject("target").setPose(targetPose);

          Translation2d error = targetPose.getTranslation().minus(pose.getTranslation());
          Distance distance = Meters.of(error.getNorm());
          SmartDashboard.putNumber("Distance (m)", distance.in(Meters));
          Rotation2d direction = error.getAngle();

          Translation2d minDirection = new Translation2d(minDistance.in(Meters), direction);
          Pose2d minPose = new Pose2d(targetPose.getTranslation().minus(minDirection), direction);
          field.getObject("min").setPose(minPose);
          Translation2d maxDirection = new Translation2d(maxDistance.in(Meters), direction);
          Pose2d maxPose = new Pose2d(targetPose.getTranslation().minus(maxDirection), direction);
          field.getObject("max").setPose(maxPose);

          if (distance.gt(maxDistance)) {
            return fieldSpeeds;
          }

          LinearVelocity assistAmount = distance.timesConversionFactor(gain);
          ChassisSpeeds assistSpeeds =
              new ChassisSpeeds(
                  assistAmount.times(direction.getCos()),
                  assistAmount.times(direction.getSin()),
                  RotationsPerSecond.zero());

          if (distance.lt(minDistance)) {
            return assistSpeeds;
          }

          // TODO In the in-between range, maybe clamp the velocity to prevent a sudden spike
          ChassisSpeeds combinedSpeeds = fieldSpeeds.plus(assistSpeeds);
          Translation2d combined =
              new Translation2d(combinedSpeeds.vxMetersPerSecond, combinedSpeeds.vyMetersPerSecond);
          double velocity = Math.min(combined.getNorm(), fieldVelocity.getNorm());
          Translation2d clamped = new Translation2d(velocity, combined.getAngle());
          return new ChassisSpeeds(
              clamped.getX(), clamped.getY(), combinedSpeeds.omegaRadiansPerSecond);
        },
        () -> targetPoseSupplier.get().getRotation());
  }
}
