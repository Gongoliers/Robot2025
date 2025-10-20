package frc.lib.outputs;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.units.measure.Voltage;
import frc.lib.values.MotorValues;
import frc.lib.values.TalonFXValues;

/**
 * A class that represents an output with two TalonFX motors controlled by voltage.
 */
public class TwoTalonFXVoltageOutput implements Output<Voltage, MotorValues> {

    /**
     * The leader TalonFX motor.
     */
    private final TalonFX leader;

    /**
     * The follower TalonFX motor.
     */
    private final TalonFX follower;

    /**
     * The output values returned by the leader TalonFX.
     */
    private final TalonFXValues leaderValues;

    /**
     * The direction of the leader motor as seen from the front of the motor.
     */
    private final InvertedValue leaderDirection;

    /**
     * Whether the follower motor should oppose the leader motor's direction.
     */
    private final boolean opposeLeader;

    /**
     * The control request used by the leader TalonFX.
     */
    private final VoltageOut control;

    /**
     * Creates an output with two TalonFX motors controlled by voltage.
     *
     * @param leader          The leader TalonFX motor.
     * @param follower        The follower TalonFX motor.
     * @param leaderDirection The direction of the leader motor as seen from the front of the motor.
     * @param opposeLeader    Whether the follower motor should oppose the leader motor's direction.
     */
    public TwoTalonFXVoltageOutput(TalonFX leader, TalonFX follower, InvertedValue leaderDirection, boolean opposeLeader) {
        this.leader = leader;
        this.follower = follower;
        this.leaderValues = new TalonFXValues(this.leader);
        this.leaderDirection = leaderDirection;
        this.opposeLeader = opposeLeader;
        this.control = new VoltageOut(0.0);
    }

    @Override
    public boolean configure() {
        TalonFXConfigurator configurator = this.leader.getConfigurator();
        MotorOutputConfigs config = new MotorOutputConfigs().withInverted(this.leaderDirection);
        StatusCode statusCode = configurator.apply(config);

        // Always configure the follower to follow the leader
        follower.setControl(new Follower(this.leader.getDeviceID(), this.opposeLeader));

        return statusCode.isOK();
    }

    @Override
    public MotorValues getOutputValues() {
        return leaderValues.refreshed();
    }

    @Override
    public void update(Voltage input) {
        leader.setControl(this.control.withOutput(input));
    }
}
