package frc.robot.auto;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.targetting.FieldTargetSupplier;
import frc.lib.targetting.ReefTarget;
import frc.robot.RobotConstants;
import frc.robot.intake.IntakeState;
import frc.robot.odometry.Odometry;
import frc.robot.ramp.RampState;
import frc.robot.superstructure.Superstructure;
import frc.robot.superstructure.SuperstructureState;
import frc.robot.swerve.Swerve;

/** Subsystem that handles all auto driving */
public class Auto extends Subsystem {

  /** Robot config */
  private RobotConfig config;

  /** Auto chooser */
  private final SendableChooser<Command> autoChooser;

  /** Odometry reference */
  private final Odometry odometry;

  /** Swerve reference */
  private final Swerve swerve;

  /** Superstructure reference */
  private final Superstructure superstructure;

  /** Translation motion profile */
  private final TrapezoidProfile translationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(2.0, 1.0));

  /** Rotation motion profile */
  private final TrapezoidProfile rotationProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(1.0, 0.5));

  public Auto(Odometry odometry, Swerve swerve, Superstructure superstructure) {
    this.odometry = odometry;
    this.swerve = swerve;
    this.superstructure = superstructure;

    try { 
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
      odometry::getPosition, 
      odometry::setPosition, 
      swerve::getRobotRelativeChassisSpeeds,
      (speeds, feedforwards) -> swerve.setChassisSpeeds(speeds), 
      new PPHolonomicDriveController(
        new PIDConstants(15, 0, 0), 
        new PIDConstants(5, 0, 0)),
      config, 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this);

    autoChooser = AutoBuilder.buildAutoChooser();

    configureAutoCommands();
  }

  private void configureAutoCommands() {
    System.out.println("did it");

    new EventTrigger("Score L4").onTrue(
      Commands.print("doing L4")
      .andThen(superstructure.superstructureTo(SuperstructureState.L4))
      .andThen(superstructure.scoreCoral())
    );

    new EventTrigger("Lift").onTrue(
      superstructure.cancelAll()
      .andThen(superstructure.superstructureTo(SuperstructureState.L1))
    );

    new EventTrigger("Score L1").onTrue(
      Commands.print("doing L1")
      .andThen(superstructure.superstructureTo(SuperstructureState.L1))
      .andThen(superstructure.intakeTo(IntakeState.CORALINFAST, RampState.STOP))
      .andThen(Commands.waitSeconds(0.7))
      .andThen(superstructure.intakeTo(IntakeState.STOP, RampState.STOP))
      .andThen(superstructure.superstructureTo(SuperstructureState.STOW))
    );

    new EventTrigger("Test L4").onTrue(
      superstructure.superstructureTo(SuperstructureState.L4)
    );

    new EventTrigger("Intake").onTrue(
      superstructure.intakeCoral()
    );
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Auto");

    tab.add("Auto chooser", autoChooser);

    tab.addBoolean("Is autonomous", AutoCoordinator::getIsAuto);
  }

  @Override
  public void periodic() {

  }

  /** Get autonomous command selected by auto chooser */
  public Command getSelectedCommand() {
    return autoChooser.getSelected();
  }

  /** Retruns a command that auto alligns to the nearest selected reef target if it is safe to do so */
  public Command allign(ReefTarget target, double safeDistance) {
    return Commands.either(
      Commands.print("Can't allign right now"),
      reefAllign(target, safeDistance),
      () -> {
        return AutoCoordinator.getIsAuto() // is currently auto
        || FieldTargetSupplier.getSafeTranslation(odometry.getPosition(), target, safeDistance).getDistance(odometry.getPosition().getTranslation()) >= 3; // is currently too far
      });
  }

  /** Returns a command that auto alligns to the nearest selected reef target */
  public Command reefAllign(ReefTarget target, double safeDistance) {
    return Commands.runOnce(() -> {
      AutoCoordinator.setIsTeleAuto(true);
      AutoCoordinator.setTargetPose(new Pose2d(
        FieldTargetSupplier.getSafeTranslation(odometry.getPosition(), target, safeDistance),
        FieldTargetSupplier.getReefFaceNormal(odometry.getPosition()).rotateBy(Rotation2d.k180deg)
      ));
    }).andThen(Commands.run(() -> {
      Pose2d currentPose = odometry.getPosition();
      Pose2d targetPose = AutoCoordinator.getTargetPose();

      Twist2d velocityTwist = odometry.getVelocity();
      double velocity = Math.hypot(velocityTwist.dx, velocityTwist.dy);

      Translation2d distanceVector = targetPose.getTranslation().minus(currentPose.getTranslation());
      double distance = distanceVector.getNorm();

      TrapezoidProfile.State targetTranslationState = translationProfile.calculate(RobotConstants.PERIODIC_DURATION,
        new TrapezoidProfile.State(distance, -velocity),
        new TrapezoidProfile.State(0, 0));

      Translation2d targetVelocityVector = distanceVector.div(distance).times(-targetTranslationState.velocity);

      TrapezoidProfile.State targetRotationState = new TrapezoidProfile.State(0, 0);
      if (!MathUtil.isNear(0.0, currentPose.getRotation().minus(targetPose.getRotation()).getDegrees(), 1)) {
        targetRotationState = rotationProfile.calculate(RobotConstants.PERIODIC_DURATION,
          new TrapezoidProfile.State(currentPose.getRotation().getRotations(), Rotation2d.fromRadians(velocityTwist.dtheta).getRotations()),
          new TrapezoidProfile.State(targetPose.getRotation().getRotations(), 0));
      }

      swerve.setChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
        targetVelocityVector.getX(),
        targetVelocityVector.getY(),
        Rotation2d.fromRotations(targetRotationState.velocity).getRadians(),
        currentPose.getRotation()));
      
    }).until(() -> {
      Pose2d currentPose = odometry.getPosition();
      Pose2d targetPose = AutoCoordinator.getTargetPose();

      return (currentPose.getTranslation().minus(targetPose.getTranslation()).getNorm() <= 0.02
        && MathUtil.isNear(0.0, currentPose.getRotation().minus(targetPose.getRotation()).getDegrees(), 1))
        || !AutoCoordinator.getIsAuto();
    }).andThen(() -> {
      AutoCoordinator.setIsTeleAuto(false);
    }));
  }

  public Command forward() {
    return Commands.runOnce(
      () -> {
        AutoCoordinator.setIsTeleAuto(true);
        swerve.setRobotRelativeChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
          0.25,
          0.0,
          0.0,
          odometry.getFieldRelativeHeading()));
      });
  }

  public Command backUp() {
    return Commands.runOnce(
      () -> {
        AutoCoordinator.setIsTeleAuto(true);
        swerve.setRobotRelativeChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
          -0.25,
          0.0,
          0.0,
          odometry.getFieldRelativeHeading()));
      });
  }

  public Command right() {
    return Commands.runOnce(
      () -> {
        AutoCoordinator.setIsTeleAuto(true);
        swerve.setRobotRelativeChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
          0.0,
          -0.25,
          0.0,
          odometry.getFieldRelativeHeading()));
      });
  }

  public Command left() {
    return Commands.runOnce(
      () -> {
        AutoCoordinator.setIsTeleAuto(true);
        swerve.setRobotRelativeChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(
          0.0,
          0.25,
          0.0,
          odometry.getFieldRelativeHeading()));
      });
  }

  public Command stop() {
    return Commands.runOnce(
      () -> {
        AutoCoordinator.setIsTeleAuto(false);
      });
  }
}
