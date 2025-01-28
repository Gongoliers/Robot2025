package frc.robot.elevator;

import frc.lib.Subsystem;

public class Elevator extends Subsystem {

  private static Elevator instance = null;

  private final LinearPositionController motor;

  private Elevator() {
    
  }

  public static Elevator getInstance() {
    if (instance == null) {
      instance = new Elevator();
    }

    return instance;
  }

  @Override
  public void initializeTab() {}


}
