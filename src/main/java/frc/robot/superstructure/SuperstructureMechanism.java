package frc.robot.superstructure;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** Class for Mechanism2d visualization of superstructure */
public class SuperstructureMechanism {

  /** Mechanism2d object */
  private final Mechanism2d mech;

  /** Root of mechanism */
  private final MechanismRoot2d root;

  /** Elevator ligament */
  private final MechanismLigament2d elevator;

  /** Manipulator ligament */
  private final MechanismLigament2d manipulator;

  /** State suppliers */
  private final Supplier<Double> elevatorPosSupplier;
  private final Supplier<Double> manipulatorPivotSupplier;

  /** Initialize Mechanism2d things */
  SuperstructureMechanism(Supplier<Double> elevatorPosSupplier, Supplier<Double> manipulatorPivotSupplier) {
    mech = new Mechanism2d(2, 2, new Color8Bit(0, 0, 0));

    root = mech.getRoot("root", 1, 0);

    elevator = root.append(new MechanismLigament2d("elevator", 0.2, 90));
    manipulator = root.append(new MechanismLigament2d("manipulator", 0.2, 0));

    this.elevatorPosSupplier = elevatorPosSupplier;
    this.manipulatorPivotSupplier = manipulatorPivotSupplier;
  }

  /** Updates Mechanism2d ligament positions each periodic loop */
  public void periodic() {
    elevator.setLength(0.2 + elevatorPosSupplier.get());
    manipulator.setAngle(manipulatorPivotSupplier.get() * 360);
  }

  /** Get sendable Mechanism2d object */
  public Mechanism2d getSendable() {
    return mech;
  }
}