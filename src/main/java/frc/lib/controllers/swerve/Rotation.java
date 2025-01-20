package frc.lib.controllers.swerve;

/** example thing */
public class Rotation {
  
  private double rotation;

  private Rotation(double rotation) {
    this.rotation = rotation;
  }

  public static Rotation fromDegrees(double degrees) {
    return new Rotation(degrees/360);
  }
  public static Rotation fromRotations(double rotations) {
    return new Rotation(rotations);
  }
  public static Rotation fromRadians(double radians) {
    return new Rotation(radians*2*Math.PI);
  }

  public double getDegrees() {
    return rotation * 360;
  }
  public double getRotations() {
    return rotation;
  }
  public double getRadians() {
    return rotation / (2 * Math.PI);
  }
}
