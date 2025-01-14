package frc.robot.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.lib.DriveRequest;
import frc.lib.Subsystem;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.MotionProfileConfig;
import frc.lib.configs.FeedbackControllerConfig.FeedbackControllerBuilder;
import frc.lib.configs.FeedforwardControllerConfig.FeedforwardControllerBuilder;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.configs.MotorConfig.MotorBuilder;
import frc.lib.controllers.swerve.SwerveModule;
import frc.robot.RobotConstants;
import frc.robot.odometry.Odometry;

import java.util.function.Function;

/** Swerve subsystem */
public class Swerve extends Subsystem {
  
  /** Swerve subsystem singleton */
  private static Swerve instance = null;

  /** Swerve modules */
  private final SwerveModule[] swerves = new SwerveModule[4];

  /** Swerve kinematics */
  private final SwerveDriveKinematics swerveKinematics;

  /** Swerve chassis yaw PID */
  private final PIDController yawPidController = new PIDController(25, 0, 0);

  /** Steer motor config */
  private final MechanismConfig steerConfig =
    MechanismBuilder.defaults()
      .motorConfig(
        MotorBuilder.defaults()
          .ccwPositive(false)
          .motorToMechRatio(150.0 / 7.0)
          .statorCurrentLimit(20.0)
          .build())
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kS(0.205)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .continuous(true)
          .kP(20.0)
          .kD(0.15)
          .tolerance(Units.degreesToRotations(1.0))
          .build())
    .build();

  /** Drive motor config */
  private final MechanismConfig driveConfig =
    MechanismBuilder.defaults()
      .motorConfig(
        MotorBuilder.defaults()
          .ccwPositive(false)
          .motorToMechRatio(6.75)
          .statorCurrentLimit(50.0)
          .build())
      .feedforwardControllerConfig(
        FeedforwardControllerBuilder.defaults()
          .kS(0.34)
          .kV(0.725)
          .build())
      .feedbackControllerConfig(
        FeedbackControllerBuilder.defaults()
          .kP(0.75)
          .build())
      .build();

  /** Wheel circumference */
  private final double wheelCircumference = Units.inchesToMeters(4.0) * Math.PI;

  /** Translation motion profile config */
  private final MotionProfileConfig translationMotionProfileConfig =
    new MotionProfileConfig(4.5, 18);

  /** Rotation motion profile config */
  private final MotionProfileConfig rotationMotionProfileConfig =
    new MotionProfileConfig(1.0, 0.0);
  
  /** Initializes the swerve subsystem and configures swerve hardware */
  private Swerve() {
    swerves[0] =
      SwerveFactory.createNorthWestModule(steerConfig, driveConfig, wheelCircumference);
    swerves[1] =
      SwerveFactory.createNorthEastModule(steerConfig, driveConfig, wheelCircumference);
    swerves[2] =
      SwerveFactory.createSouthEastModule(steerConfig, driveConfig, wheelCircumference);
    swerves[3] =
      SwerveFactory.createSouthWestModule(steerConfig, driveConfig, wheelCircumference);

    swerveKinematics = 
      new SwerveDriveKinematics(
        SwerveFactory.getNorthWestModuleTranslation(),
        SwerveFactory.getNorthEastModuleTranslation(),
        SwerveFactory.getSouthEastModuleTranslation(),
        SwerveFactory.getSouthWestModuleTranslation());

    yawPidController.enableContinuousInput(-0.5, 0.5);
    yawPidController.setTolerance(0.01);
  }

  /** 
   * Returns the swerve subsystem instance, creates a new instance if instance is null (singleton)
   * 
   * @return the swerve subsystem instance
   */
  public static Swerve getInstance() {
    if (instance == null) {
      instance = new Swerve();
    }

    return instance;
  }

  @Override
  public void periodic() {
    for (SwerveModule swerve : swerves) {
      swerve.periodic();
    }
  }

  @Override
  public void initializeTab() {
    // get shuffleboard tab
    ShuffleboardTab tab = Shuffleboard.getTab("Swerve");

    // info about swerve modules
    for (int i = 0; i < 4; i++) {
      SwerveModule swerve = swerves[i];

      // get swerve module's list widget (acts as a sub-table in NT)
      ShuffleboardLayout swerveColumn = tab.getLayout("Module " + i, BuiltInLayouts.kList);

      swerveColumn.addDouble("Angle (rot)", () -> swerve.getState().angle.getRotations());
      swerveColumn.addDouble("Velocity (mps)", () -> swerve.getState().speedMetersPerSecond);
      swerveColumn.addDouble("Setpoint Angle (deg)", () -> swerve.getSetpoint().angle.getDegrees());
      swerveColumn.addDouble("Setpoint Velocity (mps)", () -> swerve.getSetpoint().speedMetersPerSecond);
    }

    // fancy swerve states sendable
    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> swerves[0].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> swerves[0].getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> swerves[1].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> swerves[1].getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> swerves[2].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> swerves[2].getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> swerves[3].getState().angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> swerves[3].getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> Odometry.getInstance().getDriverRelativeHeading().getRadians(), null);
      }
    });
  }

  /**
   * Returns swerve kinematics
   * 
   * @return swerve kinematics
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveKinematics;
  }

  /**
   * Returns the module states
   * 
   * @return the module states
   */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      moduleStates[i] = swerves[i].getState();
    }

    return moduleStates;
  }

  /**
   * Returns the module setpoints
   * 
   * @return the module setpoints
   */
  public SwerveModuleState[] getModuleSetpoints() {
    SwerveModuleState[] moduleSetpoints = new SwerveModuleState[4];

    for (int i = 0; i < 4; i++) {
      moduleSetpoints[i] = swerves[i].getSetpoint();
    }

    return moduleSetpoints;
  }

  /**
   * Returns the module positions
   * 
   * @return the module positions
   */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];

    for (int i = 0; i < 4; i++) {
      modulePositions[i] = swerves[i].getPosition();
    }

    return modulePositions;
  }

  /**
   * Returns the chassis speeds
   * 
   * @return the chassis speeds
   */
  public ChassisSpeeds getChassisSpeeds() {
    return swerveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Sets module setpoints given desired chassis speeds
   * 
   * @param speeds chassis speeds
   */
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    speeds = ChassisSpeeds.discretize(speeds, RobotConstants.PERIODIC_DURATION);

    SwerveModuleState[] setpoints = swerveKinematics.toSwerveModuleStates(speeds);

    setSetpoints(setpoints, true);
  }

  /**
   * Sets the swerve module setpoints
   * 
   * @param setpoints setpoints
   * @param lazy if true, optimizes the module setpoints
   */
  public void setSetpoints(SwerveModuleState[] setpoints, boolean lazy) {
    SwerveDriveKinematics.desaturateWheelSpeeds(setpoints, maximumTranslationVelocity());

    for (int i = 0; i < 4; i++) {
      swerves[i].setSetpoint(setpoints[i], lazy);
    }
  }

  /**
   * Returns the maximum translation velocity
   * 
   * @return the maximum translation velocity
   */
  public double maximumTranslationVelocity() {
    return translationMotionProfileConfig.maxVelocity();
  }

  /**
   * Returns the drive radius (distance to furthest swerve module)
   * 
   * @return the drive radius (distance to furthest swerve module)
   */
  public double driveRadius() {
    return SwerveFactory.getNorthEastModuleTranslation().getNorm();
  }

  /**
   * Drives the swerve using an xbox controller
   * 
   * @param controller xbox controller
   * @return a command that drives the swerve using an xbox controller
   */
  public Command teleopDrive(CommandXboxController controller) {
    // create slew rate limiters for x and y acceleration
    final SlewRateLimiter xAccelerationLimiter =
      translationMotionProfileConfig.createAccelerationLimiter();
    final SlewRateLimiter yAccelerationLimiter =
      translationMotionProfileConfig.createAccelerationLimiter();

    // create a velocity clamping function from the rotation motion profile
    final Function<Double, Double> rotationVelocityLimiter =
      rotationMotionProfileConfig.createVelocityClamper();

    /**
     * General chassis speed limiting function
     * Takes a ChassisSpeeds object as imput, applies the acceleration and velocity clamping functions from earlier, and returns a new ChassisSpeeds object
     * 
     * Function<input_type, return_type> function_name = lambda_input -> { things done to things and returns something of typoe return_type };
     * return_type output = function_name.apply(lambda_input);
     */
    final Function<ChassisSpeeds, ChassisSpeeds> chassisSpeedsLimiter =
      chassisSpeeds -> {
        return new ChassisSpeeds(
          xAccelerationLimiter.calculate(chassisSpeeds.vxMetersPerSecond),
          yAccelerationLimiter.calculate(chassisSpeeds.vyMetersPerSecond),
          Units.rotationsToRadians(
            rotationVelocityLimiter.apply(
              Units.radiansToRotations(chassisSpeeds.omegaRadiansPerSecond))));
      };

    final Function<DriveRequest, ChassisSpeeds> chassisSpeedsGetter =
      request -> {
        double rotationVelocity;

        rotationVelocity = request.rotationVelocityAxis() * Units.rotationsToRadians(rotationMotionProfileConfig.maxVelocity());

        if (request.rotationMode() == DriveRequest.RotationMode.ALIGNING) {
          Rotation2d angleMeasurement = Odometry.getInstance().getDriverRelativeHeading();
          Rotation2d setpointAngle = request.headingAxis().getAngle();

          rotationVelocity = yawPidController.calculate(angleMeasurement.getRotations(), setpointAngle.getRotations());
        }

        return ChassisSpeeds.fromFieldRelativeSpeeds(
          request.translationAxis().getX() * translationMotionProfileConfig.maxVelocity(),
          request.translationAxis().getY() * translationMotionProfileConfig.maxVelocity(),
          rotationVelocity,
          Odometry.getInstance().getDriverRelativeHeading());
      };

    return run(
      () -> {
        setChassisSpeeds(
          chassisSpeedsLimiter.apply(
            chassisSpeedsGetter.apply(DriveRequest.fromController(controller))));
      });
  }
}
