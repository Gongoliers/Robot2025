package frc.robot.roller;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.Subsystem;
import frc.lib.motors.MotorOutput;
import frc.lib.motors.MotorOutputSim;
import frc.lib.motors.MotorValues;

public class Roller extends Subsystem {

    private final Time DT = Seconds.of(0.02);

    private final MotorOutput sim;

    private final MotorValues simValues;

    private final SysIdRoutine sysId;

    public Roller() {
        sim = new MotorOutputSim(
            Volts.per(RadiansPerSecond).ofNative(0.1),
            Volts.per(RadiansPerSecondPerSecond).ofNative(0.01),
            DCMotor.getKrakenX60(1)
        );
        simValues = new MotorValues();
        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                sim::setVoltage,
                log -> {
                    log
                        .motor("roller")
                        .voltage(simValues.motorVoltage)
                        .angularPosition(simValues.position)
                        .angularVelocity(simValues.velocity)
                        .angularAcceleration(simValues.acceleration);
                },
                this
            )
        );
    }

    @Override
    public void initializeTab() {
        ShuffleboardTab tab = Shuffleboard.getTab("Roller");

        ShuffleboardLayout simColumn = tab.getLayout(
            "Sim",
            BuiltInLayouts.kList
        );
        simColumn.addNumber("Motor Voltage (V)", () ->
            simValues.motorVoltage.in(Volts)
        );
        simColumn.addNumber("Motor Current (A)", () ->
            simValues.statorCurrent.in(Amps)
        );
        simColumn.addNumber("Velocity (rot per sec)", () ->
            simValues.velocity.in(RotationsPerSecond)
        );
        simColumn.addNumber("Acceleration (rot per sec per sec)", () ->
            simValues.acceleration.in(RotationsPerSecondPerSecond)
        );
    }

    @Override
    public void periodic() {
        sim.updateValues(simValues, DT);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }
}
