package frc.lib.swerves;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Time;

public class IdealSwerveSim implements SwerveOutput {

  private final Time DT = Seconds.of(0.02);

  private final SwerveDrivetrain.SwerveDriveState state;

  public IdealSwerveSim() {
    this.state = new SwerveDrivetrain.SwerveDriveState();
  }

  @Override
  public void setControl(SwerveRequest request) {
    if (request instanceof SwerveRequest.FieldCentric) {
      handleFieldCentric((SwerveRequest.FieldCentric) request);
    }

    if (request instanceof SwerveRequest.FieldCentricFacingAngle) {
      handleFieldCentricFacingAngle((SwerveRequest.FieldCentricFacingAngle) request);
    }
  }

  private void handleFieldCentric(SwerveRequest.FieldCentric request) {
    double vx = request.VelocityX;
    double vy = request.VelocityY;
    double omega = request.RotationalRate;
    double dt = DT.in(Seconds);

    double x = state.Pose.getX() + vx * dt;
    double y = state.Pose.getY() + vy * dt;
    double angle = state.Pose.getRotation().getRadians() + omega * dt;

    state.Pose = new Pose2d(x, y, Rotation2d.fromRadians(angle));
  }

  private void handleFieldCentricFacingAngle(SwerveRequest.FieldCentricFacingAngle request) {
    double vx = request.VelocityX;
    double vy = request.VelocityY;
    double dt = DT.in(Seconds);

    double x = state.Pose.getX() + vx * dt;
    double y = state.Pose.getY() + vy * dt;

    state.Pose = new Pose2d(x, y, request.TargetDirection);
  }

  @Override
  public SwerveDrivetrain.SwerveDriveState getState() {
    return state;
  }

  public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionStdDevs) {
    // No-op for simulation, pose estimates are fully trusted in sim
  }

  @Override
  public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds) {
    state.Pose = visionPose;
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> visionStdDevs) {
    addVisionMeasurement(visionPose, timestampSeconds);
  }

  @Override
  public void setOperatorPerspectiveForward(Rotation2d fieldDirection) {
    // TODO No-op for simulation, since we are always driving relative to the field
  }

  @Override
  public void resetPose(Pose2d newPose) {
    state.Pose = newPose;
  }

  @Override
  public void resetRotation(Rotation2d newRotation) {
    state.Pose = new Pose2d(state.Pose.getTranslation(), newRotation);
  }

  @Override
  public void resetTranslation(Translation2d newTranslation) {
    state.Pose = new Pose2d(newTranslation, state.Pose.getRotation());
  }
}
