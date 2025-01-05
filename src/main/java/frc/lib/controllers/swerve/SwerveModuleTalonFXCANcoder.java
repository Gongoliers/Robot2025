package frc.lib.controllers.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.controllers.position.PositionController;
import frc.lib.controllers.position.PositionController.PositionControllerValues;
import frc.lib.controllers.velocity.VelocityController;
import frc.lib.controllers.velocity.VelocityController.VelocityControllerValues;

/** Swerve module with TalonFX motor controllers (steer and drive) and a CTRE CANcoder */
public class SwerveModuleTalonFXCANcoder implements SwerveModule {

  /** Steer motor */
  private final PositionController steerMotor;

  /** Steer motor values */
  private final PositionControllerValues steerMotorValues = new PositionControllerValues();

  /** Drive motor */
  private final VelocityController driveMotor;

  /** Drive motor values */
  private final VelocityControllerValues driveMotorValues = new VelocityControllerValues();

  /** Module wheel circumference */
  private final double wheelCircumference;

  /** Moduule setpoint */
  private SwerveModuleState setpoint;

  public SwerveModuleTalonFXCANcoder(
      PositionController steerMotor, VelocityController driveMotor, double wheelCircumference) {
    
    this.steerMotor = steerMotor;
    this.steerMotor.configure();

    this.driveMotor = driveMotor;
    this.driveMotor.configure();

    this.wheelCircumference = wheelCircumference;

    setpoint = new SwerveModuleState();
  }

  @Override
  public SwerveModuleState getState() {
    steerMotor.getUpdatedVals(steerMotorValues);
    driveMotor.getUpdatedVals(driveMotorValues);

    return new SwerveModuleState(
      driveMotorValues.velRotationsPerSec * wheelCircumference,
      Rotation2d.fromRotations(steerMotorValues.posRotations));
  }

  @Override
  public SwerveModuleState getSetpoint() {
    return setpoint;
  }

  @Override
  public void setSetpoint(SwerveModuleState setpoint, boolean lazy) {
    SwerveModuleState state = getState();

    setpoint.optimize(state.angle);

    // if lazy, do some more optimization
    if (lazy) {
      // scale speed by the module's error (distance of current angle from setpoint angle)
      Rotation2d error = setpoint.angle.minus(state.angle);
      setpoint.speedMetersPerSecond *= error.getCos();
    }

    steerMotor.setSetpoint(setpoint.angle.getRotations(), 0);
    driveMotor.setSetpoint(setpoint.speedMetersPerSecond / wheelCircumference);

    this.setpoint = setpoint;
  }

  @Override
  public SwerveModulePosition getPosition() {
    steerMotor.getUpdatedVals(steerMotorValues);
    driveMotor.getUpdatedVals(driveMotorValues);

    return new SwerveModulePosition(
      driveMotorValues.posRotations * wheelCircumference,
      Rotation2d.fromRotations(steerMotorValues.posRotations));
  }
}