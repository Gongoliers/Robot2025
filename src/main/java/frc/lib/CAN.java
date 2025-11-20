package frc.lib;

import java.util.Objects;

/** CAN class that makes using multiple buses simpler */
public record CAN(int id, String bus) {

  /**
   * Creates a CAN identifier for a device
   *
   * @param id CAN id of device
   * @param bus CAN bus name
   */
  public CAN {
    Objects.requireNonNull(id);
    Objects.requireNonNull(bus);
  }

  /**
   * CAN constructor that uses the default bus name, an empty string
   *
   * @param id CAN id of device
   */
  public CAN(int id) {
    this(id, "");
  }
}
