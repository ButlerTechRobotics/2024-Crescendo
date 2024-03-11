// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.intake;

import frc.robot.subsystems.rollers.GenericRollerSystemIOSparkFlex;

public class IntakeIOSparkFlex extends GenericRollerSystemIOSparkFlex implements IntakeIO {
  private static final double reduction = (1.0 / 1.0);
  private static final int id = 25;
  private static final int currentLimitAmps = 40;
  private static final boolean inverted = true;

  public IntakeIOSparkFlex() {
    super(id, currentLimitAmps, inverted, false, reduction);
  }
}
