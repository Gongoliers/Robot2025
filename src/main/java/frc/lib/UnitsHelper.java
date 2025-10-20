package frc.lib;

import edu.wpi.first.units.*;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;

import static edu.wpi.first.units.Units.*;

public class UnitsHelper {

    public static Per<AngleUnit, DistanceUnit> inverseConversionFactor(Per<DistanceUnit, AngleUnit> conversionFactor) {
        return Rotations.of(1 / conversionFactor.in(Meters.per(Rotation))).per(Meter);
    }

    public static Per<AngularVelocityUnit, VoltageUnit> kV_rps(Per<LinearVelocityUnit, VoltageUnit> kV_mps, Per<DistanceUnit, AngleUnit> conversionFactor) {
        return RotationsPerSecond.of(kV_mps.in(MetersPerSecond.per(Volt)) * UnitsHelper.inverseConversionFactor(conversionFactor).in(Rotations.per(Meter))).per(Volt);
    }

    public static Per<AngularAccelerationUnit, VoltageUnit> kA_rpsps(Per<LinearAccelerationUnit, VoltageUnit> kA_mpsps, Per<DistanceUnit, AngleUnit> conversionFactor) {
        return RotationsPerSecondPerSecond.of(kA_mpsps.in(MetersPerSecondPerSecond.per(Volt)) * UnitsHelper.inverseConversionFactor(conversionFactor).in(Rotations.per(Meter))).per(Volt);
    }

    public static Per<VoltageUnit, AngularVelocityUnit> inverseKv(Per<AngularVelocityUnit, VoltageUnit> kV) {
        return Volts.of(1 / kV.in(RotationsPerSecond.per(Volt))).per(RotationsPerSecond);
    }

    public static Per<VoltageUnit, AngularAccelerationUnit> inverseKa(Per<AngularAccelerationUnit, VoltageUnit> kA) {
        return Volts.of(1 / kA.in(RotationsPerSecondPerSecond.per(Volt))).per(RotationsPerSecondPerSecond);
    }

    public static Per<LinearVelocityUnit, AngularVelocityUnit> velocityConversionFactor(Per<DistanceUnit, AngleUnit> conversionFactor) {
        return MetersPerSecond.of(conversionFactor.in(Meters.per(Rotation))).per(RotationsPerSecond);
    }

    public static Per<LinearAccelerationUnit, AngularAccelerationUnit> accelerationConversionFactor(Per<DistanceUnit, AngleUnit> conversionFactor) {
        return MetersPerSecondPerSecond.of(conversionFactor.in(Meters.per(Rotation))).per(RotationsPerSecondPerSecond);
    }

}
