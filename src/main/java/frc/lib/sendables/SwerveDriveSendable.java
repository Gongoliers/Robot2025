package frc.lib.sendables;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import java.util.function.Supplier;

/**
 * Swerve drive state sendable that allows fancy visualization in most dashboards and simpler
 * looking swerve logging
 */
public class SwerveDriveSendable implements Sendable {

  /** Gets swerve module states [NW, NE, SW, SE] */
  private final Supplier<SwerveModuleState[]> statesSupplier;

  /** Gets robot rotation */
  private final Supplier<Rotation2d> robotAngleSupplier;

  /**
   * Constructs a swerve drive state sendable given a module states supplier and robot angle
   * supplier
   *
   * @param statesSupplier module states supplier [NW, NE, SW, SE]
   * @param robotAngleSupplier robot angle supplier
   */
  public SwerveDriveSendable(
      Supplier<SwerveModuleState[]> statesSupplier, Supplier<Rotation2d> robotAngleSupplier) {

    this.statesSupplier = statesSupplier;
    this.robotAngleSupplier = robotAngleSupplier;
  }

  /**
   * Constructs a swerve drive state sendable given 4 module state suppliers and a robot angle
   * supplier
   *
   * @param NWStateSupplier NW module state supplier
   * @param NEStateSupplier NE module state supplier
   * @param SWStateSupplier SW module state supplier
   * @param SEStateSupplier SE module state supplier
   * @param robotAngleSupplier robot angle supplier
   */
  public SwerveDriveSendable(
      Supplier<SwerveModuleState> NWStateSupplier,
      Supplier<SwerveModuleState> NEStateSupplier,
      Supplier<SwerveModuleState> SWStateSupplier,
      Supplier<SwerveModuleState> SEStateSupplier,
      Supplier<Rotation2d> robotAngleSupplier) {

    this.statesSupplier =
        () -> {
          SwerveModuleState[] states = {
            NWStateSupplier.get(),
            NEStateSupplier.get(),
            SWStateSupplier.get(),
            SEStateSupplier.get()
          };
          return states;
        };
    this.robotAngleSupplier = robotAngleSupplier;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveDrive");

    builder.addDoubleProperty(
        "Front Left Angle", () -> statesSupplier.get()[0].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Left Velocity", () -> statesSupplier.get()[0].speedMetersPerSecond, null);

    builder.addDoubleProperty(
        "Front Right Angle", () -> statesSupplier.get()[1].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Front Right Velocity", () -> statesSupplier.get()[1].speedMetersPerSecond, null);

    builder.addDoubleProperty(
        "Back Left Angle", () -> statesSupplier.get()[2].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Back Left Velocity", () -> statesSupplier.get()[2].speedMetersPerSecond, null);

    builder.addDoubleProperty(
        "Back Right Angle", () -> statesSupplier.get()[3].angle.getRadians(), null);
    builder.addDoubleProperty(
        "Back Right Velocity", () -> statesSupplier.get()[3].speedMetersPerSecond, null);

    builder.addDoubleProperty("Robot Angle", () -> robotAngleSupplier.get().getRadians(), null);
  }
}
