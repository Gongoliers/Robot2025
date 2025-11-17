package frc.lib.swerves;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Time;

import static edu.wpi.first.units.Units.Seconds;

public class IdealSwerveSim implements Swerve {

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

}
