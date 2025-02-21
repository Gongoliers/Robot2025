package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PPLTVController;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.Subsystem;
import frc.robot.RobotConstants;
import frc.robot.elevator.ElevatorState;
import frc.robot.odometry.Odometry;
import frc.robot.superstructure.Superstructure;
import frc.robot.swerve.Swerve;

/** Handles choosing and running autos */
public class Auto extends Subsystem {

  /** Auto singleton */
  private static Auto instance = null;
  
  /** Auto chooser  */
  private final SendableChooser<Command> autoChooser;

  /** Odometry reference */
  private final Odometry odometry;

  /** Superstructure reference */
  private final Superstructure superstructure;

  /** Swerve reference */
  private final Swerve swerve;

  /** Robot config */
  private RobotConfig config;

  /** Initialize auto subsystem */
  private Auto() {
    odometry = Odometry.getInstance();
    superstructure = Superstructure.getInstance();
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
        new PIDConstants(5.0, 0.0, 0.0),
        new PIDConstants(5.0, 0.0, 0.0)
      ), 
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

    addEventTriggers();
  }

  /** Get auto instance */
  public static Auto getInstance() {
    if (instance == null) {
      instance = new Auto();
    }

    return instance;
  }

  @Override
  public void periodic() {}

  @Override
  public void initializeTab() {
    // Get tab
    ShuffleboardTab tab = Shuffleboard.getTab("Auto");

    tab.add(autoChooser);
  }

  /** Adds event triggers that schedule commands across the robot */
  public void addEventTriggers() {
    new EventTrigger("STOW").onTrue(superstructure.elevatorTo(ElevatorState.STOW));
    new EventTrigger("L2").onTrue(superstructure.elevatorTo(ElevatorState.L2));
  }

  public Command getSelectedCommand() {
    return autoChooser.getSelected();
  }
}
