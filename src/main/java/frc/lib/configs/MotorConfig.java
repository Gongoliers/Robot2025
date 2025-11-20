package frc.lib.configs;

/**
 * Motor config
 *
 * @param neutralBrake true means the motor will brake when given no voltage
 * @param ccwPositive true means positive voltage rotates the motor ccw
 * @param rotorToSensorRatio ratio of rotor rotations to sensor rotations
 * @param sensorTOMechRatio ratio of sensor rotations to mechanism rotations
 * @param statorCurrentLimit current limit in the stator
 * @param supplyCurrentLimit supply current limit
 */
public record MotorConfig(
    boolean neutralBrake,
    boolean ccwPositive,
    double rotorToSensorRatio,
    double sensorToMechRatio,
    double statorCurrentLimit,
    double supplyCurrentLimit) {

  /** Easier and more modular way to construct a motor config */
  public static class MotorBuilder {
    private boolean neutralBrake;
    private boolean ccwPositive;
    private double rotorToSensorRatio;
    private double sensorToMechRatio;
    private double statorCurrentLimit;
    private double supplyCurrentLimit;

    private MotorBuilder(
        boolean neutralBrake,
        boolean ccwPositive,
        double rotorToSensorRatio,
        double sensorToMechRatio,
        double statorCurrentLimit,
        double supplyCurrentLimit) {
      this.neutralBrake = neutralBrake;
      this.ccwPositive = ccwPositive;
      this.rotorToSensorRatio = rotorToSensorRatio;
      this.sensorToMechRatio = sensorToMechRatio;
      this.statorCurrentLimit = statorCurrentLimit;
      this.supplyCurrentLimit = supplyCurrentLimit;
    }

    /**
     * Returns a builder with default values
     *
     * @return a builder with default values
     */
    public static MotorBuilder defaults() {
      return new MotorBuilder(false, true, 1.0, 1.0, 80.0, 40.0);
    }

    /**
     * Returns a builder with values copied from the input config
     *
     * @param config config to be copied
     * @return a builder with values copied from the input config
     */
    public static MotorBuilder edit(MotorConfig config) {
      return new MotorBuilder(
          config.neutralBrake(),
          config.ccwPositive(),
          config.rotorToSensorRatio(),
          config.sensorToMechRatio(),
          config.statorCurrentLimit(),
          config.supplyCurrentLimit());
    }

    public MotorBuilder neutralBrake(boolean neutralBrake) {
      this.neutralBrake = neutralBrake;
      return this;
    }

    public MotorBuilder ccwPositive(boolean ccwPositive) {
      this.ccwPositive = ccwPositive;
      return this;
    }

    public MotorBuilder rotorToSensorRatio(double rotorToSensorRatio) {
      this.rotorToSensorRatio = rotorToSensorRatio;
      return this;
    }

    public MotorBuilder sensorToMechRatio(double sensorToMechRatio) {
      this.sensorToMechRatio = sensorToMechRatio;
      return this;
    }

    public MotorBuilder statorCurrentLimit(double statorCurrentLimit) {
      this.statorCurrentLimit = statorCurrentLimit;
      return this;
    }

    public MotorBuilder supplyCurrentLimit(double supplyCurrentLimit) {
      this.supplyCurrentLimit = supplyCurrentLimit;
      return this;
    }

    /**
     * Returns the builder as a config with private immutable values
     *
     * @return the builder as a config with private immutable values
     */
    public MotorConfig build() {
      return new MotorConfig(
          this.neutralBrake,
          this.ccwPositive,
          this.rotorToSensorRatio,
          this.sensorToMechRatio,
          this.statorCurrentLimit,
          this.supplyCurrentLimit);
    }
  }
}
