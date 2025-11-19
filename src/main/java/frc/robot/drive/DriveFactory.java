package frc.robot.drive;

import frc.lib.swerves.SwerveOutput;
import frc.robot.generated.TunerConstants;

public class DriveFactory {

    public static SwerveOutput createSwerve() {
        return TunerConstants.createDrivetrain();
    }

}
