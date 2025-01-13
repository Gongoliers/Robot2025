package frc.robot.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.lib.CAN;
import frc.lib.configs.MechanismConfig;
import frc.lib.configs.AbsoluteEncoderConfig.AbsoluteEncoderBuilder;
import frc.lib.configs.MechanismConfig.MechanismBuilder;
import frc.lib.controllers.position.PositionController;
import frc.lib.controllers.position.PositionControllerSim;
import frc.lib.controllers.position.PositionControllerTalonFXSteer;
import frc.lib.controllers.swerve.SwerveModule;
import frc.lib.controllers.swerve.SwerveModuleTalonFXCANcoder;
import frc.lib.controllers.velocity.VelocityController;
import frc.lib.controllers.velocity.VelocityControllerSim;
import frc.lib.controllers.velocity.VelocityControllerTalonFXPIDF;
import frc.robot.Robot;

/** Creates swerve hardware */
public class SwerveFactory {
  
  /**
   * Creates a generic swerve module
   * 
   * @param steer steer motor CAN
   * @param azimuth azimuth encoder CAN
   * @param drive drive motor CAN
   * @param steerConfig steer motor config
   * @param driveConfig drive motor config
   * @param wheelCircumference circumference of the swerve module's wheel
   * @return a generic swerve module
   */
  private static SwerveModule createModule(
      CAN steer,
      CAN azimuth,
      CAN drive,
      MechanismConfig steerConfig,
      MechanismConfig driveConfig,
      double wheelCircumference) {
    
    return new SwerveModuleTalonFXCANcoder(
      createSteerMotor(steer, azimuth, steerConfig),
      createDriveMotor(drive, driveConfig),
      wheelCircumference);
  }

  /** 
   * Creates the north west swerve module
   * 
   * @param steerConfig steer motor config
   * @param driveConfig drive motor config
   * @param wheelCircumference circumference of the swerve module's wheel
   * @return the north west swerve module
   */
  public static SwerveModule createNorthWestModule(
      MechanismConfig steerConfig,
      MechanismConfig driveConfig,
      double wheelCircumference) {
    
    return createModule(
      new CAN(8, "swerve"),
      new CAN(9, "swerve"),
      new CAN(10, "swerve"),
      MechanismBuilder.edit(steerConfig)
        .absoluteEncoderConfig(
          AbsoluteEncoderBuilder.edit(steerConfig.absoluteEncoderConfig())
            .offset(Rotation2d.fromRotations(0.183837).unaryMinus())
            .build())
        .build(),
      driveConfig,
      wheelCircumference);
  }

  /**
   * Gets the north west swerve module's translation
   * 
   * @return the north west swerve module's tranlsation
   */
  public static Translation2d getNorthWestModuleTranslation() {
    return new Translation2d(Units.inchesToMeters(10.375), Units.inchesToMeters(10.375));
  }

  /** 
   * Creates the north east swerve module
   * 
   * @param steerConfig steer motor config
   * @param driveConfig drive motor config
   * @param wheelCircumference circumference of the swerve module's wheel
   * @return the north east swerve module
   */
  public static SwerveModule createNorthEastModule(
      MechanismConfig steerConfig,
      MechanismConfig driveConfig,
      double wheelCircumference) {
    
    return createModule(
      new CAN(16, "swerve"),
      new CAN(17, "swerve"),
      new CAN(18, "swerve"),
      MechanismBuilder.edit(steerConfig)
        .absoluteEncoderConfig(
          AbsoluteEncoderBuilder.edit(steerConfig.absoluteEncoderConfig())
            .offset(Rotation2d.fromRotations(0.243408).unaryMinus())
            .build())
        .build(),
      driveConfig,
      wheelCircumference);
  }

  /**
   * Gets the north east swerve module's translation
   * 
   * @return the north east swerve module's tranlsation
   */
  public static Translation2d getNorthEastModuleTranslation() {
    return new Translation2d(Units.inchesToMeters(10.375), Units.inchesToMeters(-10.375));
  }

  /** 
   * Creates the south east swerve module
   * 
   * @param steerConfig steer motor config
   * @param driveConfig drive motor config
   * @param wheelCircumference circumference of the swerve module's wheel
   * @return the south east swerve module
   */
  public static SwerveModule createSouthEastModule(
      MechanismConfig steerConfig,
      MechanismConfig driveConfig,
      double wheelCircumference) {
    
    return createModule(
      new CAN(24, "swerve"),
      new CAN(25, "swerve"),
      new CAN(26, "swerve"),
      MechanismBuilder.edit(steerConfig)
        .absoluteEncoderConfig(
          AbsoluteEncoderBuilder.edit(steerConfig.absoluteEncoderConfig())
            .offset(Rotation2d.fromRotations(0.3786).unaryMinus())
            .build())
        .build(),
      driveConfig,
      wheelCircumference);
  }

  /**
   * Gets the south east swerve module's translation
   * 
   * @return the south east swerve module's tranlsation
   */
  public static Translation2d getSouthEastModuleTranslation() {
    return new Translation2d(Units.inchesToMeters(-10.375), Units.inchesToMeters(-10.375));
  }

  /** 
   * Creates the south west swerve module
   * 
   * @param steerConfig steer motor config
   * @param driveConfig drive motor config
   * @param wheelCircumference circumference of the swerve module's wheel
   * @return the south west swerve module
   */
  public static SwerveModule createSouthWestModule(
      MechanismConfig steerConfig,
      MechanismConfig driveConfig,
      double wheelCircumference) {
    
    return createModule(
      new CAN(32, "swerve"),
      new CAN(33, "swerve"),
      new CAN(34, "swerve"),
      MechanismBuilder.edit(steerConfig)
        .absoluteEncoderConfig(
          AbsoluteEncoderBuilder.edit(steerConfig.absoluteEncoderConfig())
            .offset(Rotation2d.fromRotations(-0.43164).unaryMinus())
            .build())
        .build(),
      driveConfig,
      wheelCircumference);
  }

  /**
   * Gets the south west swerve module's translation
   * 
   * @return the south west swerve module's tranlsation
   */
  public static Translation2d getSouthWestModuleTranslation() {
    return new Translation2d(Units.inchesToMeters(-10.375), Units.inchesToMeters(10.375));
  }

  /**
   * Creates a steer motor
   * 
   * @param steer steer motor CAN
   * @param azimuth azimuth encoder CAN
   * @param config steer motor conrig
   * @return a steer motor
   */
  private static PositionController createSteerMotor(
      CAN steer, CAN azimuth, MechanismConfig config) {
    
    if (Robot.isReal()) {
      return new PositionControllerTalonFXSteer(steer, azimuth, config, false);
    }

    return new PositionControllerSim();
  }

  /**
   * Creates a drive motor
   * 
   * @param drive drive motor CAN
   * @param config drive motor config
   * @return a drive motor
   */
  private static VelocityController createDriveMotor(CAN drive, MechanismConfig config) {
    if (Robot.isReal()) {
      return new VelocityControllerTalonFXPIDF(drive, config, false);
    }

    return new VelocityControllerSim();
  }
}
