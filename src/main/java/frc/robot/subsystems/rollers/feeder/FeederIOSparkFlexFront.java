package frc.robot.subsystems.rollers.feeder;

import frc.robot.subsystems.rollers.GenericRollerSystemIOSparkFlex;

public class FeederIOSparkFlexFront extends GenericRollerSystemIOSparkFlex implements FeederIO {
  private static final double reduction = (1.0 / 1.0);
  private static final int id = 24;
  private static final int currentLimitAmps = 40;
  private static final boolean inverted = true;

  public FeederIOSparkFlexFront() {
    super(id, currentLimitAmps, inverted, true, reduction);
  }
}
