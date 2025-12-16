package frc.lib.swerves;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface SwerveOutput {

  void setControl(SwerveRequest request);

  SwerveDrivetrain.SwerveDriveState getState();

  void setVisionMeasurementStdDevs(Matrix<N3, N1> visionStdDevs);

  void addVisionMeasurement(Pose2d visionPose, double timestampSeconds);

  void addVisionMeasurement(
      Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> visionStdDevs);

  void setOperatorPerspectiveForward(Rotation2d fieldDirection);

  void resetPose(Pose2d newPose);

  void resetRotation(Rotation2d newRotation);

  void resetTranslation(Translation2d newTranslation);
}
