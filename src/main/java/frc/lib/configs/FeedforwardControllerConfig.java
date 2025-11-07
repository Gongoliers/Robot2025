package frc.lib.configs;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

/**
 * Feedforward config
 *
 * @param kS voltage to overcome static friction
 * @param kG voltage to overcome gravity
 * @param kV voltage to overcome friction or drag that reduces velocity
 * @param kA voltage to overcome inertia or other resistive forces that reduce acceleration
 */
public record FeedforwardControllerConfig<U extends Unit, U_PER_SEC extends PerUnit<U, TimeUnit>, U_PER_SEC_PER_SEC extends PerUnit<U_PER_SEC, TimeUnit>>(
        Voltage kS,
        Voltage kG,
        Measure<PerUnit<VoltageUnit, U_PER_SEC>> kV,
        Measure<PerUnit<VoltageUnit, U_PER_SEC_PER_SEC>> kA,
        U Unit,
        U_PER_SEC VelocityUnit,
        U_PER_SEC_PER_SEC AccelerationUnit) {


    public FeedforwardControllerConfig(Voltage kS, Voltage kG, Measure<PerUnit<VoltageUnit, U_PER_SEC>> kV, Measure<PerUnit<VoltageUnit, U_PER_SEC_PER_SEC>> kA, U unit) {
        this(kS, kG, kV, kA, unit, (U_PER_SEC) unit.per(Second), (U_PER_SEC_PER_SEC) unit.per(Second).per(Second));
    }


    /**
     * Construct simple feedforward without kG
     *
     * @param kS voltage to overcome static friction
     * @param kV voltage to overcome friction or drag that reduces velocity
     * @param kA voltage to overcome inertia or other resistive forces that reduce acceleration
     */
    public FeedforwardControllerConfig(Voltage kS, Measure<PerUnit<VoltageUnit, U_PER_SEC>> kV, Measure<PerUnit<VoltageUnit, U_PER_SEC_PER_SEC>> kA, U unit) {
        this(kS, Volts.of(0), kV, kA, unit);
    }

    /**
     * Creates a simple motor feedforward using this config
     *
     * @return a simple motor feedforward using this config
     */
    public SimpleMotorFeedforward createSimpleMotorFeedforward() {
        return new SimpleMotorFeedforward(kS.in(Volts), kV.in(Volts.per(VelocityUnit)), kA.in(Volts.per(AccelerationUnit)));
    }

    /**
     * Creates an arm feedforward using this config
     *
     * @return an arm feedforward using this config
     */
    public ArmFeedforward createArmFeedforward() {
        return new ArmFeedforward(kS.in(Volts), kG.in(Volts), kV.in(Volts.per(VelocityUnit)), kA.in(Volts.per(AccelerationUnit)));
    }

    /**
     * Creates an elevator feedforward using this config
     *
     * @return an elevator feedforward using this config
     */
    public ElevatorFeedforward createElevatorFeedforward() {
        return new ElevatorFeedforward(kS.in(Volts), kG.in(Volts), kV.in(Volts.per(VelocityUnit)), kA.in(Volts.per(AccelerationUnit)));
    }

}
