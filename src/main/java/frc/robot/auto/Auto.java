package frc.robot.auto;

import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
import frc.robot.odometry.Odometry;
import frc.robot.swerve.Swerve;

/** Subsystem that handles all auto driving */
public class Auto extends Subsystem {
  
  /** Instance of auto subsystem */
  private static Auto instance = null;

  /** Robot config */
  private RobotConfig config;

  /** Auto chooser */
  private final SendableChooser<Command> autoChooser;

  /** Odometry reference */
  private final Odometry odometry;

  /** Swerve reference */
  private final Swerve swerve;

  public static Auto getInstance() {
    if (instance == null) {
      instance = new Auto();
    }

    return instance;
  }

  private Auto() {
    odometry = Odometry.getInstance();
    swerve = Swerve.getInstance();

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
      odometry::getPosition, 
      odometry::setPosition, 
      swerve::getChassisSpeeds, 
      (speeds, feedforwards) -> swerve.setChassisSpeeds(speeds), 
      new PPHolonomicDriveController(
        new PIDConstants(5, 0, 0), 
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
    new EventTrigger("BEGIN PATH").onTrue(Commands.runOnce(() -> AutoCoordinator.setIsTeleAuto(true)));
    new EventTrigger("END PATH").onTrue(Commands.runOnce(() -> AutoCoordinator.setIsTeleAuto(false)));
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

  /**
   * Gets a pathfinding command to a reef target with some safe distance
   * 
   * @param target field target (left right center)
   * @param safeDistance distance in meters to keep between the target position and chassis rails
   * @return a pathfinding command to a reef target with some safe distance
   */
  private static Supplier<Command> getPathingCommandSupplier(ReefTarget target, double safeDistance) {
    return () -> {
      Pose2d currentPose = Odometry.getInstance().getPosition();
      Rotation2d reefFaceNormal = FieldTargetSupplier.getReefFaceNormal(currentPose);
      Pose2d targetPose = new Pose2d(
        new Translation2d(
          target.getForwardOffset() + RobotConstants.CHASSIS_SIDE_LENGTH*0.0254/2 + safeDistance,
          target.getHorizontalOffset())
          .rotateBy(reefFaceNormal)
          .plus(FieldTargetSupplier.getReefCenter()),
        reefFaceNormal.rotateBy(Rotation2d.k180deg));

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(
          currentPose.getTranslation(),
          targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle()),
        targetPose);

      PathConstraints constraints = new PathConstraints(2, 2, 1*Math.PI, 2*Math.PI);

      PathPlannerPath path = new PathPlannerPath(waypoints, constraints, null, new GoalEndState(0.0, targetPose.getRotation()));
      path.preventFlipping = true;

      return AutoBuilder.followPath(path);
    };
  }

  /**
   * Gets a command that follows a generated path to the nearest selected reef target
   * 
   * @param target reef target (left right center)
   * @param safeDistance distance in meters to keep between the target and the cahssis of the robot
   * @return a command that follows a generated path to the nearest selected reef target
   */
  public Command pathfindToTarget(ReefTarget target, double safeDistance) {
    return Commands.defer(getPathingCommandSupplier(target, safeDistance), Set.of(this))
      .alongWith(Commands.runOnce(() -> AutoCoordinator.setIsTeleAuto(true)))
      .andThen(() -> AutoCoordinator.setIsTeleAuto(false));
  }
}
