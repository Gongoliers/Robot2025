package frc.robot.targetting;

public enum ReefTarget {
  LEFT(0.8255, -0.1651),
  RIGHT(0.8255, 0.0),
  CENTER(0.8255, 0.1651);

  private final double offsetForward;
  private final double offsetHorizontal;

  ReefTarget(double offsetForward, double offsetHorizontal) {
    this.offsetForward = offsetForward;
    this.offsetHorizontal = offsetHorizontal;
  }

  public double getForwardOffset() {
    return offsetForward;
  }

  public double getHorizontalOffset() {
    return offsetHorizontal;
  }
}
