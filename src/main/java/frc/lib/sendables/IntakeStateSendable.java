package frc.lib.sendables;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.intake.IntakeState;

/** Nice ElevatorState sendable */
public class IntakeStateSendable implements Sendable {
  
  private final Supplier<IntakeState> stateSupplier;

  /**
   * Creates an ElevatorState sendable 
   * 
   * @param stateSupplier function that supplies ElevatorState enum
   */
  public IntakeStateSendable(Supplier<IntakeState> stateSupplier) {
    this.stateSupplier = stateSupplier;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ElevatorState");

    builder.addStringProperty("Name", () -> stateSupplier.get().name(), null);
    builder.addDoubleProperty("Vel (mps)", () -> stateSupplier.get().getVelRotationsPerSec(), null);
  }
}
