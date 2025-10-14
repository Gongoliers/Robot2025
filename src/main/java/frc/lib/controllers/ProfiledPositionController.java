package frc.lib.controllers;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Time;

/**
 * Represents a controller for a mechanism that reaches a position using
 * a profile of velocity setpoints.
 * <p>
 * This controller does not require any sensor information from the controller
 * responsible for reaching the velocity setpoints, so the type wildcard '?'
 * is used to allow the velocity controller to return any values. There aren't
 * any sensors used by this controller, so there isn't a clear choice for values.
 * Returning when the setpoint is at the goal indicates when the profile is completed,
 * which may be useful for some applications.
 */
public class ProfiledPositionController implements Controller<Distance, Boolean> {

    // Represents the default update frequency of 50 Hertz
    private static final Time kDt = Seconds.of(0.02);

    private final Controller<AngularVelocity, ?> controller;

    private final TrapezoidProfile profile;

    private TrapezoidProfile.State goal;

    private TrapezoidProfile.State setpoint;

    private final Per<AngleUnit, DistanceUnit> conversionFactor;

    public ProfiledPositionController(
            Controller<AngularVelocity, ?> controller,
            LinearVelocity velocityConstraint,
            LinearAcceleration accelerationConstraint,
            Distance initialPosition,
            Per<AngleUnit, DistanceUnit> conversionFactor) {
        this.controller = controller;
        this.profile =
                new TrapezoidProfile(
                        new TrapezoidProfile.Constraints(
                                velocityConstraint.in(MetersPerSecond),
                                accelerationConstraint.in(MetersPerSecondPerSecond)));
        this.goal = new TrapezoidProfile.State(initialPosition.in(Meters), 0.0);
        this.setpoint = this.goal;
        this.conversionFactor = conversionFactor;
    }

    @Override
    public boolean configure() {
        return controller.configure();
    }

    @Override
    public Boolean getValues() {
        // '.equals' compares the position and velocity of each state
        return this.setpoint.equals(this.goal);
    }

    @Override
    public void update(Distance goal) {
        this.goal = new TrapezoidProfile.State(goal.in(Meters), 0.0);
        this.setpoint =
                this.profile.calculate(kDt.in(Seconds), this.setpoint, this.goal);
        var linearVelocity = MetersPerSecond.of(this.setpoint.velocity);
        var metersToRotations = conversionFactor.in(Rotations.per(Meter));
        var angularVelocity = RotationsPerSecond.of(linearVelocity.in(MetersPerSecond) * metersToRotations);
        controller.update(angularVelocity);
    }
}
