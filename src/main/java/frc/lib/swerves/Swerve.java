package frc.lib.swerves;

import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;

public interface Swerve {

    void setControl(SwerveRequest request);

    SwerveDrivetrain.SwerveDriveState getState();

}
