package frc.robot.subsystems.rollers.intake;

import frc.robot.subsystems.rollers.GenericRollerSystemIOSparkFlex;

public class IntakeIOSparkFlex extends GenericRollerSystemIOSparkFlex implements IntakeIO {
  private static final double reduction = (1.0 / 1.0);
  private static final int id = 25;
  private static final int currentLimitAmps = 80;
  private static final boolean inverted = true;

  public IntakeIOSparkFlex() {
    super(id, currentLimitAmps, inverted, false, reduction);
  }
}
