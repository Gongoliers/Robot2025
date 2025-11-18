package frc.robot.drive;

import frc.lib.swerves.IdealSwerveSim;
import frc.lib.swerves.SwerveOutput;

public class DriveFactory {

    public static SwerveOutput createSwerve() {
        return new IdealSwerveSim();
    }

}
