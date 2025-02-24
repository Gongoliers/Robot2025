package frc.robot.auto;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.EventMarker;
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

  public static Auto getInstance() {
    if (instance == null) {
      instance = new Auto();
    }

    return instance;
  }

  private Auto() {
    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      e.printStackTrace();
    }

    AutoBuilder.configure(
      Odometry.getInstance()::getPosition, 
      Odometry.getInstance()::setPosition, 
      Swerve.getInstance()::getChassisSpeeds, 
      (speeds, feedforwards) -> Swerve.getInstance().setChassisSpeeds(speeds), 
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
    new EventTrigger("BEGIN PATH").onTrue(Commands.runOnce(() -> AutoHandler.setIsTeleAuto(true)));
    new EventTrigger("END PATH").onTrue(Commands.runOnce(() -> AutoHandler.setIsTeleAuto(false)));
  }

  @Override
  public void initializeTab() {
    ShuffleboardTab tab = Shuffleboard.getTab("Auto");

    tab.add("Auto chooser", autoChooser);

    tab.addBoolean("Is autonomous", AutoHandler::getIsAuto);
  }

  @Override
  public void periodic() {
    
  }

  /** Get autonomous command selected by auto chooser */
  public Command getSelectedCommand() {
    return autoChooser.getSelected();
  }

  /**
   * Gets a command that follows a generated path to the nearest selected reef target
   * 
   * @param target reef target (left right center)
   * @param safeDistance distance in meters to keep between the target and the cahssis of the robot
   * @return a command that follows a generated path to the nearest selected reef target
   */
  public Command pathfindToTarget(ReefTarget target, double safeDistance) {
    Supplier<Translation2d> targetTranslation = () -> FieldTargetSupplier.getSafeTranslation(Odometry.getInstance().getPosition(), target, safeDistance);
    Supplier<Rotation2d> targetRotation = () -> FieldTargetSupplier.getReefFaceNormal(Odometry.getInstance().getPosition()).rotateBy(Rotation2d.k180deg);
    PathConstraints constraints = new PathConstraints(3, 3, 2 * Math.PI, 4 * Math.PI);

    Supplier<Command> pathfindSupplier = () -> AutoBuilder.pathfindToPose(new Pose2d(targetTranslation.get(), targetRotation.get()), constraints, 0.0)
      .alongWith(Commands.runOnce(() -> AutoHandler.setIsTeleAuto(true)))
      .andThen(() -> AutoHandler.setIsTeleAuto(false));

    return Commands.defer(pathfindSupplier, Set.of(this, Swerve.getInstance()));
  }
}
