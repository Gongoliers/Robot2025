package frc.robot.roller;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.CAN;
import frc.lib.Subsystem;
import frc.lib.configs.MotorConfig;
import frc.lib.motors.*;

import java.util.function.Supplier;

public class Roller extends Subsystem {

    private final Time DT = Seconds.of(0.02);

    private final MotorOutput motor;

    private final MotorValues values;

    private final MotorOutput sim;

    private final MotorValues simValues;

    private final SysIdRoutine sysId;

    public Roller() {
        motor = new MotorOutputTalonFX(MotorConfig.MotorBuilder.defaults().build(), new CAN(0));
        values = new MotorValues();

        sim = new LossyMotorOutputSim(new MotorOutputSim(
            Volts.per(RadiansPerSecond).ofNative(0.1),
            Volts.per(RadiansPerSecondPerSecond).ofNative(0.01),
            DCMotor.getKrakenX60(1)
        ), Volts.of(0.14));
        simValues = new MotorValues();

        sysId = new SysIdRoutine(
            new SysIdRoutine.Config(),
            new SysIdRoutine.Mechanism(
                motor::setVoltage,
                log -> log
                    .motor("roller")
                    .voltage(values.motorVoltage)
                    .angularPosition(values.position)
                    .angularVelocity(values.velocity)
                    .angularAcceleration(values.acceleration),
                this
            )
        );
    }

    @Override
    public void initializeTab() {
        ShuffleboardTab tab = Shuffleboard.getTab("Roller");

        ShuffleboardLayout motorColumn = tab.getLayout(
                "Motor",
                BuiltInLayouts.kList
        );
        motorColumn.addNumber("Motor Voltage (V)", () ->
                values.motorVoltage.in(Volts)
        );
        motorColumn.addNumber("Motor Current (A)", () ->
                values.statorCurrent.in(Amps)
        );
        motorColumn.addNumber("Velocity (rot per sec)", () ->
                values.velocity.in(RotationsPerSecond)
        );
        motorColumn.addNumber("Acceleration (rot per sec per sec)", () ->
                values.acceleration.in(RotationsPerSecondPerSecond)
        );

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
        motor.updateValues(values, DT);
        sim.updateValues(simValues, DT);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysId.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysId.dynamic(direction);
    }

    private void setVoltage(Voltage voltage) {
        motor.setVoltage(voltage);
        sim.setVoltage(voltage);
    }

    public Command runVoltage(Supplier<Voltage> voltage) {
        return run(() -> setVoltage(voltage.get())).finallyDo(() -> setVoltage(Volts.zero()));
    }

    public Command runVoltage(Voltage voltage) {
        return runVoltage(() -> voltage);
    }
}
