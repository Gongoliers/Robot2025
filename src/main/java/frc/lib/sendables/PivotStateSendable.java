package frc.lib.sendables;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.pivot.PivotState;

/** Nice ElevatorState sendable */
public class PivotStateSendable implements Sendable {
  
  private final Supplier<PivotState> stateSupplier;

  /**
   * Creates an ElevatorState sendable 
   * 
   * @param stateSupplier function that supplies ElevatorState enum
   */
  public PivotStateSendable(Supplier<PivotState> stateSupplier) {
    this.stateSupplier = stateSupplier;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ElevatorState");

    builder.addStringProperty("Name", () -> stateSupplier.get().name(), null);
    builder.addDoubleProperty("Pos (m)", () -> stateSupplier.get().getPosRotations(), null);
  }
}
