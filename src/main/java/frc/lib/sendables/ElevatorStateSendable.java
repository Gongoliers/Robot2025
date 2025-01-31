package frc.lib.sendables;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.elevator.ElevatorState;

/** Nice ElevatorState sendable */
public class ElevatorStateSendable implements Sendable {
  
  private final Supplier<ElevatorState> stateSupplier;

  /**
   * Creates an ElevatorState sendable 
   * 
   * @param stateSupplier function that supplies ElevatorState enum
   */
  public ElevatorStateSendable(Supplier<ElevatorState> stateSupplier) {
    this.stateSupplier = stateSupplier;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("ElevatorState");

    builder.addStringProperty("Name: ", () -> stateSupplier.get().name(), null);
    builder.addDoubleProperty("Pos (m): ", () -> stateSupplier.get().getPosMeters(), null);
    builder.addDoubleProperty("Vel (mps): ", () -> stateSupplier.get().getVelMetersPerSec(), null);
  }
}
