package frc.robot.odometry;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.Subsystem;
import frc.lib.sensors.Gyroscope;
import frc.lib.sensors.Gyroscope.GyroscopeValues;
import frc.lib.targetting.Limelights;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;

/** Odometry subsystem */
public class Odometry extends Subsystem {
  
  /** Gyroscope */
  private final Gyroscope gyroscope;

  /** Gyroscope values */
  private final GyroscopeValues gyroscopeValues = new GyroscopeValues();

  /** Limelights for field pose estimation */
  private final Limelights limelights;

  /** Supplies swerve module positions */
  private Supplier<SwerveModulePosition[]> modulePositionsSupplier = () -> new SwerveModulePosition[4];

  /** Supplies swerve chassis speeds */
  private Supplier<ChassisSpeeds> chassisSpeedsSupplier = () -> new ChassisSpeeds();

  /** Swerve drive pose estimator */
  private final SwerveDrivePoseEstimator poseEstimator;

  /** Field */
  private final Field2d field;

  /** Initializes odometry subsystem and odometry hardware */
  public Odometry(SwerveDrivePoseEstimator poseEstimator) {
    gyroscope = OdometryFactory.createGyroscope(this);
    gyroscope.configure();

    this.poseEstimator = poseEstimator;
    poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
	// TODO: Seed the gyroscope with the initial pose yaw?

    limelights = OdometryFactory.createLimelights();
    limelights.addLimelight("limelight-east", 
      0.2715768,
      0.0809498,
      0.1539494,
      5,
      25,
      30);
    limelights.addLimelight("limelight-west",
      0.2715768,
      -0.0812292,
      0.1539494,
      5,
      25,
      -30);

    field = new Field2d();
  }

  public void setModulePositionsSupplier(Supplier<SwerveModulePosition[]> modulePositionsSupplier) {
    this.modulePositionsSupplier = modulePositionsSupplier;
  }

  public void setChassisSpeedsSupplier(Supplier<ChassisSpeeds> chassisSpeedsSupplier) {
    this.chassisSpeedsSupplier = chassisSpeedsSupplier;
  }

  @Override
  public void periodic() {
    gyroscope.periodic();
    gyroscope.getUpdatedVals(gyroscopeValues);

    limelights.addVisionMeasurements(poseEstimator, Rotation2d.fromRotations(gyroscopeValues.yawRotations));

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

    // make field widget
    tab.add("Field", this.field);
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
    return getFieldRelativeHeading().rotateBy(
      (Robot.isRedAlliance())
        ? Rotation2d.k180deg
        : Rotation2d.kZero);
  }
  
  /**
   * Sets the position of the robot on the field
   * used for testing and the start of matches
   * 
   * @param position the position of the robot on the field
   */
  public void setPosition(Pose2d position) {
    System.out.println("setPosition");
    poseEstimator.resetPosition(
      Rotation2d.fromRotations(gyroscopeValues.yawRotations), 
      modulePositionsSupplier.get(), 
      position);
  }

  public void forcePosition(Pose2d position) {
    System.out.println("forcePosition");
    poseEstimator.resetPosition(
      position.getRotation(),
      modulePositionsSupplier.get(),
      position
    );

    setRotation(position.getRotation());
  }

  /**
   * Sets the rotation of the robot at its current position on the field
   * used for testing and the start of matches
   * 
   * @param rotation the position of the robot on the field
   */
  public void setRotation(Rotation2d rotation) {
    System.out.println("setRotation");
    Pose2d position = getPosition();

    setPosition(new Pose2d(position.getTranslation(), rotation));
  }

  /**
   * Gets a mt1 vision measurement from a given camera and fully resets pose estimator position to whatever the measurement says
   * 
   * @param camera limelight name
   * @return a command that gets a mt1 vision measurement from a given camera and fully resets pose estimator position to whatever the measurement says
   */
  public Command trustVisionMeasurement(String camera) {
    return Commands.runOnce(() -> {
      LimelightHelpers.PoseEstimate poseEstimate = limelights.getVisionMeasurement(camera);

      forcePosition(poseEstimate.pose);
    });
  }

  /**
   * Zeroes the yaw of the pigeon2 gyroscope
   * used for convenience while testing and reallignment mid-match
   * 
   * @return a command that zeros the yaw of the pigeon2 gyroscope
   */
  public Command setYaw(double yawRotations) {
    return Commands.runOnce(
      () -> {
        gyroscope.setYaw(yawRotations);
        setRotation(Rotation2d.fromRotations(yawRotations));
      }).andThen(Commands.print("setYaw"));
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
