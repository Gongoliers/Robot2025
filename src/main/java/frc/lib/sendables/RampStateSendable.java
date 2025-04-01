package frc.lib.sendables;

import java.util.function.Supplier;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import frc.robot.ramp.RampState;

/** Nice ElevatorState sendable */
public class RampStateSendable implements Sendable {
  
  private final Supplier<RampState> stateSupplier;

  /**
   * Creates an ElevatorState sendable 
   * 
   * @param stateSupplier function that supplies ElevatorState enum
   */
  public RampStateSendable(Supplier<RampState> stateSupplier) {
    this.stateSupplier = stateSupplier;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("RampState");

    builder.addStringProperty("Name", () -> stateSupplier.get().name(), null);
    builder.addDoubleProperty("Vel (mps)", () -> stateSupplier.get().getVelRotationsPerSec(), null);
  }
}
