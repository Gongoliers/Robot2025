package frc.lib.controllers;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.*;
import frc.lib.MotorOutputValues;

import static edu.wpi.first.units.Units.*;

/**
 * Represents a controller for a mechanism that reaches a position using
 * a profile of velocity setpoints.
 */
public class ProfiledPositionController implements Controller<Angle, MotorOutputValues> {

    // Represents the default update frequency of 50 Hertz
    private static final Time DT = Seconds.of(0.02);

    private final Controller<AngularVelocity, MotorOutputValues> controller;

    private final TrapezoidProfile profile;

    private TrapezoidProfile.State goal;

    private TrapezoidProfile.State setpoint;

    public ProfiledPositionController(
            Controller<AngularVelocity, MotorOutputValues> controller,
            AngularVelocity velocityConstraint,
            AngularAcceleration accelerationConstraint,
            Angle initialPosition) {
        this.controller = controller;
        this.profile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                velocityConstraint.in(RotationsPerSecond),
                                accelerationConstraint.in(RotationsPerSecondPerSecond)));
        this.goal = new TrapezoidProfile.State(initialPosition.in(Rotations), 0.0);
        this.setpoint = this.goal;
    }

    @Override
    public boolean configure() {
        return controller.configure();
    }

    @Override
    public MotorOutputValues getOutputValues() {
        return this.controller.getOutputValues();
    }

    @Override
    public void update(Angle goal) {
        this.goal = new TrapezoidProfile.State(goal.in(Rotations), 0.0);
        this.setpoint =
                this.profile.calculate(DT.in(Seconds), this.setpoint, this.goal);
        var angularVelocity = RotationsPerSecond.of(this.setpoint.velocity);
        this.controller.update(angularVelocity);
    }
}
