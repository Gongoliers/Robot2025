package frc.lib.sendables;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.endgame.EndgameState;

/** Nice ElevatorState sendable */
public class EndgameStateSendable implements Sendable {
  
  private final Supplier<EndgameState> stateSupplier;

  /**
   * Creates an ElevatorState sendable 
   * 
   * @param stateSupplier function that supplies ElevatorState enum
   */
  public EndgameStateSendable(Supplier<EndgameState> stateSupplier) {
    this.stateSupplier = stateSupplier;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("EndgameState");

    builder.addStringProperty("Name", () -> stateSupplier.get().name(), null);
    builder.addDoubleProperty("Pos (m)", () -> stateSupplier.get().getPosRotations(), null);
  }
}
