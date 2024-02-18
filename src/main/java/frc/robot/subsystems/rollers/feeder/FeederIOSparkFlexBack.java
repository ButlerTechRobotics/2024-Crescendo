package frc.robot.subsystems.rollers.feeder;

import frc.robot.subsystems.rollers.GenericRollerSystemIOSparkFlex;

public class FeederIOSparkFlexBack extends GenericRollerSystemIOSparkFlex implements FeederIO {
  private static final double reduction = (1.0 / 1.0);
  private static final int id = 21;
  private static final int currentLimitAmps = 80;
  private static final boolean inverted = false;

  public FeederIOSparkFlexBack() {
    super(id, currentLimitAmps, inverted, true, reduction);
  }
}
