package frc.robot.odometry;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.CAN;
import frc.lib.Subsystem;
import frc.lib.sensors.Gyroscope;
import frc.lib.sensors.Gyroscope.GyroscopeValues;
import frc.lib.targetting.Limelights;
import frc.robot.swerve.Swerve;

/** Odometry subsystem */
public class Odometry extends Subsystem {
  
  /** Odometry subsystem singleton */
  private static Odometry instance = null;

  /** Gyroscope */
  private final Gyroscope gyroscope;

  /** Gyroscope values */
  private final GyroscopeValues gyroscopeValues = new GyroscopeValues();

  /** Limelights for field pose estimation */
  private final Limelights limelights;

  /** Supplies swerve module positions */
  private final Supplier<SwerveModulePosition[]> modulePositionsSupplier;

  /** Supplies swerve chassis speeds */
  private final Supplier<ChassisSpeeds> chassisSpeedsSupplier;

  /** Swerve drive pose estimator */
  private final SwerveDrivePoseEstimator poseEstimator;

  /** Field */
  private final Field2d field;

  /** Initializes odometry subsystem and odometry hardware */
  private Odometry() {
    gyroscope = OdometryFactory.createGyroscope(this, new CAN(0));
    gyroscope.configure();

    modulePositionsSupplier = () -> Swerve.getInstance().getModulePositions();
    chassisSpeedsSupplier = () -> Swerve.getInstance().getChassisSpeeds();

    gyroscope.getUpdatedVals(gyroscopeValues);

    poseEstimator =
      new SwerveDrivePoseEstimator(
        Swerve.getInstance().getKinematics(), 
        Rotation2d.fromRotations(gyroscopeValues.yawRotations), 
        modulePositionsSupplier.get(), 
        new Pose2d());
    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));

    limelights = OdometryFactory.createLimelights();
    limelights.addLimelights("limelight1", "limelight2");

    field = new Field2d();
  }

  /** 
   * Returns the odometry subsystem instance, creates a new instance if instance is null (singleton)
   * 
   * @return the odometry subsystem instance
   */
  public static Odometry getInstance() {
    if (instance == null) {
      instance = new Odometry();
    }

    return instance;
  }

  @Override
  public void periodic() {
    gyroscope.periodic();
    gyroscope.getUpdatedVals(gyroscopeValues);

    limelights.addVisionMeasurements(poseEstimator);

    poseEstimator.update(
      Rotation2d.fromRotations(gyroscopeValues.yawRotations), 
      modulePositionsSupplier.get());

    field.setRobotPose(getPosition());
  }

  @Override
  public void initializeTab() {
    // get shuffleboard tab
    ShuffleboardTab tab = Shuffleboard.getTab("Odometry");

    // make position list widget (list widgets will act as sub-tables in NT)
    ShuffleboardLayout position = tab.getLayout("Position", BuiltInLayouts.kList);

    position.addDouble("X (m)", () -> getPosition().getX());
    position.addDouble("Y (m)", () -> getPosition().getY());
    position.addDouble("Field Rotation (deg)", () -> getFieldRelativeHeading().getDegrees());
    position.addDouble("Driver Rotation (deg)", () -> getDriverRelativeHeading().getDegrees());

    // make velocity list widget
    ShuffleboardLayout velocity = tab.getLayout("Velocity", BuiltInLayouts.kList);

    velocity.addDouble("X Velocity (mps)", () -> getVelocity().dx);
    velocity.addDouble("Y Velocity (mps)", () -> getVelocity().dy);
    velocity.addDouble("Angular Velocity (dps)", () -> Units.radiansToDegrees((getVelocity().dtheta)));
    velocity.addDouble("Velocity", () -> Math.hypot(getVelocity().dx, getVelocity().dy));

    // make field list widget
    ShuffleboardLayout field = tab.getLayout("Field", BuiltInLayouts.kList);

    field.add("Field", this.field);
  }

  /**
   * Returns the position of the robot on the field
   * 
   * @return the position of the robot on the field
   */
  public Pose2d getPosition() {
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns the rotation of the robot on the field where zero is pointing away from the blue alliance wall
   * 
   * @return the rotation of the robot on the field where zero is pointing away from the blue alliance wall
   */
  public Rotation2d getFieldRelativeHeading() {
    return getPosition().getRotation();
  }

  /**
   * Returns the rotation of the robot on the field where zero is away from the driver's alliance wall
   * 
   * @return the rotation of the robot on the field where zero is away from the driver's alliance wall
   */
  public Rotation2d getDriverRelativeHeading() {
    return Rotation2d.fromRotations(gyroscopeValues.yawRotations);
  }

  /**
   * Sets the position of the robot on the field
   * used for testing and the start of matches
   * 
   * @param position the position of the robot on the field
   */
  public void setPosition(Pose2d position) {
    poseEstimator.resetPosition(
      Rotation2d.fromRotations(gyroscopeValues.yawRotations), 
      modulePositionsSupplier.get(), 
      position);
  }

  /**
   * Sets the rotation of the robot at its current position on the field
   * used for testing and the start of matches
   * 
   * @param rotation the position of the robot on the field
   */
  public void setRotation(Rotation2d rotation) {
    Pose2d position = getPosition();

    setPosition(new Pose2d(position.getTranslation(), rotation));
  }

  /**
   * Zeroes the yaw of the pigeon2 gyroscope
   * used for convenience while testing and reallignment mid-match
   * 
   * @return a command that zeros the yaw of the pigeon2 gyroscope
   */
  public Command zeroYaw() {
    return Commands.runOnce(
      () -> {
        gyroscope.setYaw(0.0);
        setRotation(Rotation2d.fromDegrees(0.0));
      });
  }

  /**
   * Gets the velocity of the robot on the field
   * 
   * @return the velocity of the robot on the vield
   */
  public Twist2d getVelocity() {
    // Prevent errors from calling before swerve pose estimator is initialized
    if (poseEstimator == null) return new Twist2d();

    ChassisSpeeds chassisSpeeds = chassisSpeedsSupplier.get();
    Rotation2d rotation = getPosition().getRotation();

    double xVelMetersPerSecond =
      chassisSpeeds.vxMetersPerSecond * rotation.getCos()
        - chassisSpeeds.vyMetersPerSecond * rotation.getSin();
    double yVelMetersPerSecond =
      chassisSpeeds.vxMetersPerSecond * rotation.getSin()
        + chassisSpeeds.vyMetersPerSecond * rotation.getCos();

    return new Twist2d(
      xVelMetersPerSecond, yVelMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
  }
}
