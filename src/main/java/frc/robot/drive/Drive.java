package frc.robot.drive;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Subsystem;
import frc.lib.swerves.SwerveOutput;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

public class Drive extends Subsystem {

    private final SwerveOutput swerve;

    private SwerveDrivetrain.SwerveDriveState state;

    private final Field2d field;

    private boolean hasSetPerspective = false;

    public Drive(SwerveOutput swerve) {
        this.swerve = swerve;
        this.state = new SwerveDrivetrain.SwerveDriveState();
        this.field = new Field2d();
    }

    @Override
    public void initializeTab() {
        ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

        tab.add(field);
        tab.addDoubleArray("States", () -> {
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
        tab.addDoubleArray("Targets", () -> {
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

        // NOTE This was taken from the generated project, unsure if it is needed
        // trySettingPerspective();
    }

    private void trySettingPerspective() {
        if (!hasSetPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                swerve.setOperatorPerspectiveForward(
                        allianceColor == DriverStation.Alliance.Red
                                ? Rotation2d.k180deg
                                : Rotation2d.kZero
                );
                hasSetPerspective = true;
            });
        }
    }

    public Pose2d getPose() {
        return state.Pose;
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

    public Command sysIdQuasistatic(
        SysIdRoutine routine,
        SysIdRoutine.Direction direction
    ) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(
        SysIdRoutine routine,
        SysIdRoutine.Direction direction
    ) {
        return routine.dynamic(direction);
    }

    public Command drive(Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        // TODO Make factory for requests
        SwerveRequest.FieldCentric request = new SwerveRequest.FieldCentric();

        return run(() -> {
            ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
            swerve.setControl(
                request
                    .withVelocityX(fieldSpeeds.vxMetersPerSecond)
                    .withVelocityY(fieldSpeeds.vyMetersPerSecond)
                    .withRotationalRate(fieldSpeeds.omegaRadiansPerSecond)
            );
        });
    }

    public Command driveFacing(
        Supplier<ChassisSpeeds> fieldSpeedsSupplier,
        Supplier<Rotation2d> directionSupplier
    ) {
        // TODO Make factory for requests
        SwerveRequest.FieldCentricFacingAngle request =
            new SwerveRequest.FieldCentricFacingAngle();

        return run(() -> {
            ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
            Rotation2d direction = directionSupplier.get();
            swerve.setControl(
                request
                    .withVelocityX(fieldSpeeds.vxMetersPerSecond)
                    .withVelocityY(fieldSpeeds.vyMetersPerSecond)
                    .withTargetDirection(direction)
            );
        });
    }

    public Command driveToward(Supplier<ChassisSpeeds> fieldSpeedsSupplier, Supplier<Pose2d> targetPoseSupplier) {
        final Per<LinearVelocityUnit, DistanceUnit> GAIN = MetersPerSecond.of(4).per(Meter);

        // TODO Make utility class with closures for mutations
       return driveFacing(() -> {
           ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
           Pose2d pose = getPose();
           Pose2d targetPose = targetPoseSupplier.get();

           Translation2d error = targetPose.getTranslation().minus(pose.getTranslation());
           Distance distance = Meters.of(error.getNorm());
           Rotation2d direction = error.getAngle();

           LinearVelocity assist = distance.timesConversionFactor(GAIN);
           ChassisSpeeds assistSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(assist.times(direction.getCos()), assist.times(direction.getSin()), RotationsPerSecond.zero(), pose.getRotation());

            return fieldSpeeds.plus(assistSpeeds);
       }, () -> targetPoseSupplier.get().getRotation());
    }
}
