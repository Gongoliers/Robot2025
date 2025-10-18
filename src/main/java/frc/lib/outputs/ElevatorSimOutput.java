package frc.lib.outputs;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.lib.values.MotorValues;

import static edu.wpi.first.units.Units.*;

public class ElevatorSimOutput implements Output<Voltage, MotorValues> {

    private final ElevatorSim sim;

    private final Per<AngleUnit, DistanceUnit> positionRatio;

    private final Per<AngularVelocityUnit, LinearVelocityUnit> velocityRatio;

    private Voltage voltage = Volts.zero();

    public ElevatorSimOutput(Per<VoltageUnit, AngularVelocityUnit> kV, Per<VoltageUnit, AngularAccelerationUnit> kA, DCMotor motor, Per<AngleUnit, DistanceUnit> ratio) {
        this(new ElevatorSim(kV.in(Volts.per(RotationsPerSecond)) * ratio.in(Rotations.per(Meter)), kA.in(Volts.per(RotationsPerSecondPerSecond)) * ratio.in(Rotations.per(Meter)), motor, Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY, false, 0), ratio);
    }

    public ElevatorSimOutput(ElevatorSim sim, Per<AngleUnit, DistanceUnit> ratio) {
        this.sim = sim;
        this.positionRatio = ratio;
        this.velocityRatio = RotationsPerSecond.per(MetersPerSecond).ofNative(ratio.in(Rotations.per(Meter)));
    }

    @Override
    public boolean configure() {
        return true;
    }

    @Override
    public MotorValues getOutputValues() {
        MotorValues motorValues = new MotorValues();
        motorValues.position = Meters.of(sim.getPositionMeters()).timesConversionFactor(positionRatio);
        motorValues.velocity = MetersPerSecond.of(sim.getVelocityMetersPerSecond()).timesConversionFactor(velocityRatio);
        motorValues.motorVoltage = voltage;
        motorValues.statorCurrent = Amps.of(sim.getCurrentDrawAmps());
        return motorValues;
    }

    @Override
    public void update(Voltage input) {
        this.voltage = input;
        sim.setInputVoltage(input.in(Volts));
        sim.update(0.02);
    }
}
