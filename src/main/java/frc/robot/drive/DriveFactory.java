package frc.robot.drive;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Subsystem;
import frc.lib.swerves.PhoenixSwerve;
import frc.lib.swerves.SwerveOutput;

public class DriveFactory {

  public static SwerveOutput createSwerve() {
    return new PhoenixSwerve(
        TunerConstants.DrivetrainConstants,
        TunerConstants.FrontLeft,
        TunerConstants.FrontRight,
        TunerConstants.BackLeft,
        TunerConstants.BackRight);
  }

  protected static SysIdRoutine createDriveRoutine(SwerveOutput swerve, Subsystem subsystem) {
    SwerveRequest.SysIdSwerveTranslation m_translationCharacterization =
        new SwerveRequest.SysIdSwerveTranslation();

    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> swerve.setControl(m_translationCharacterization.withVolts(output)),
            null,
            subsystem));
  }

  protected static SysIdRoutine createSteerRoutine(SwerveOutput swerve, Subsystem subsystem) {
    SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization =
        new SwerveRequest.SysIdSwerveSteerGains();

    return new SysIdRoutine(
        new SysIdRoutine.Config(
            null, // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
        new SysIdRoutine.Mechanism(
            volts -> swerve.setControl(m_steerCharacterization.withVolts(volts)), null, subsystem));
  }

  protected static SysIdRoutine createRotationRoutine(SwerveOutput swerve, Subsystem subsystem) {
    SwerveRequest.SysIdSwerveRotation m_rotationCharacterization =
        new SwerveRequest.SysIdSwerveRotation();

    return new SysIdRoutine(
        new SysIdRoutine.Config(
            /* This is in radians per second², but SysId only supports "volts per second" */
            Volts.of(Math.PI / 6).per(Second),
            /* This is in radians per second, but SysId only supports "volts" */
            Volts.of(Math.PI),
            null, // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
        new SysIdRoutine.Mechanism(
            output -> {
              /* output is actually radians per second, but SysId only supports "volts" */
              swerve.setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
              /* also log the requested output for SysId */
              SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
            },
            null,
            subsystem));
  }
}
