package frc.robot.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.NTDouble;
import frc.lib.Subsystem;
import frc.lib.sendables.SwerveDriveSendable;
import frc.lib.swerves.SwerveOutput;
import java.util.function.Supplier;

public class Drive extends Subsystem {

  private final SwerveOutput swerve;

  private SwerveDrivetrain.SwerveDriveState state;

  private final Field2d field;

  private Supplier<Pose2d> targetSupplier;

  private boolean hasSetPerspective = false;

  private final DriverAssistance driverAssistance;

  public Drive(SwerveOutput swerve) {
    this.swerve = swerve;
    this.state = new SwerveDrivetrain.SwerveDriveState();
    this.field = new Field2d();
    this.driverAssistance =
        new DriverAssistance(MetersPerSecond.per(Meter).ofNative(8), Meters.of(1), Meters.of(2));
    this.targetSupplier = Pose2d::new;
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    tab.add("Field", field);
    tab.add(
        "States",
        new SwerveDriveSendable(() -> state.ModuleStates, () -> this.getPose().getRotation()));
    tab.add(
        "Targets",
        new SwerveDriveSendable(() -> state.ModuleTargets, () -> this.getPose().getRotation()));
  }

  @Override
  public void periodic() {
    state = swerve.getState();
    field.setRobotPose(state.Pose);
    driverAssistance.drawDebugObjects(field, getPose(), getTargetPose());
    SmartDashboard.putBoolean(
        "Dead Spots?", driverAssistance.hasDeadSpots(TunerConstants.kSpeedAt12Volts));

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

  public Pose2d getTargetPose() {
    return targetSupplier.get();
  }

  public void setTargetPose(Supplier<Pose2d> targetPose) {
    targetSupplier = targetPose;
  }

  public void setTargetPose(Pose2d targetPose) {
    setTargetPose(() -> targetPose);
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

    final NTDouble<PerUnit<AngularVelocityUnit, AngleUnit>> KP =
        new NTDouble<>("DriveFacing.KP", RotationsPerSecond.per(Rotation));
    final NTDouble<AngularVelocityUnit> MAX_ROTATIONAL_RATE =
        new NTDouble<>("DriveFacing.MaxRotationalRate", RotationsPerSecond);

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

  public Command driveToTarget(Supplier<ChassisSpeeds> fieldSpeeds) {
    return driveFacing(
        () -> driverAssistance.applyDriverAssistance(fieldSpeeds.get(), getPose(), getTargetPose()),
        () -> getTargetPose().getRotation());
  }
}
