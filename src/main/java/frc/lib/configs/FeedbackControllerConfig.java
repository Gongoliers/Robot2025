package frc.lib.configs;

import edu.wpi.first.math.controller.PIDController;

/** 
 * Feedback controller config
 * 
 * @param kP proportional voltage
 * @param kI integral voltage
 * @param kD derivative voltage
 * @param continuous true enables continuous input
 * @param tolerance position tolerance
 * @param rateTolerance velocity tolerance
 */
public record FeedbackControllerConfig(
    double kP,
    double kI,
    double kD,
    boolean continuous,
    double tolerance,
    double rateTolerance) {

  /**
   * Creates a new PID controller using this config
   * 
   * @return a new PID controller using this config
   */
  public PIDController createPIDController() {
    final PIDController pidController = new PIDController(kP, kI, kD);

    pidController.setTolerance(tolerance, rateTolerance);

    if (continuous) {
      pidController.enableContinuousInput(-0.5, 0.5);
    }

    return pidController;
  }

  /** Easier and more modular way to construct feedback controller configs */
  public static class FeedbackControllerBuilder {
    private double kP;
    private double kI;
    private double kD;
    private boolean continuous;
    private double tolerance;
    private double rateTolerance;
    
    private FeedbackControllerBuilder(
        double kP,
        double kI,
        double kD,
        boolean continuous,
        double tolerance,
        double rateTolerance) {
      this.kP = kP;
      this.kI = kI;
      this.kD = kD;
      this.continuous = continuous;
      this.tolerance = tolerance;
      this.rateTolerance = rateTolerance;
    }

    /** 
     * Returns a builder with default values
     * 
     * @return a builder with default values
     */
    public static FeedbackControllerBuilder defaults() {
      return new FeedbackControllerBuilder(
        0.0, 
        0.0, 
        0.0, 
        false, 
        0.0, 
        0.0);
    }

    /** 
     * Returns a builder with values copied from the input config
     * 
     * @param config config to be copied
     * @return a builder with values copied from the input config
     */
    public static FeedbackControllerBuilder edit(FeedbackControllerConfig config) {
      return new FeedbackControllerBuilder(
        config.kP(), 
        config.kI(), 
        config.kD(), 
        config.continuous(), 
        config.tolerance(), 
        config.rateTolerance());
    }

    public FeedbackControllerBuilder kP(double kP) {
      this.kP = kP;
      return this;
    }

    public FeedbackControllerBuilder kI(double kI) {
      this.kI = kI;
      return this;
    }

    public FeedbackControllerBuilder kD(double kD) {
      this.kD = kD;
      return this;
    }

    public FeedbackControllerBuilder continuous(boolean continuous) {
      this.continuous = continuous;
      return this;
    }

    public FeedbackControllerBuilder tolerance(double tolerance) {
      this.tolerance = tolerance;
      return this;
    }

    public FeedbackControllerBuilder rateTolerance(double rateTolerance) {
      this.rateTolerance = rateTolerance;
      return this;
    }

    /**
     * Returns the builder as a config with private immutable values
     * 
     * @return the builder as a config with private immutable values
     */
    public FeedbackControllerConfig build() {
      return new FeedbackControllerConfig(
        this.kP, 
        this.kI, 
        this.kD, 
        this.continuous, 
        this.tolerance, 
        this.rateTolerance);
    }
  }
}
