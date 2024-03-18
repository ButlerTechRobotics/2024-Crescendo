// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.intake;

import frc.robot.Constants;
import frc.robot.subsystems.rollers.GenericRollerSystemIOKrakenFOC;

public class IntakeIOKrakenFOC extends GenericRollerSystemIOKrakenFOC implements IntakeIO {
  private static final int id;
  private static final String bus = "*";
  private static final int currentLimitAmps = 40;
  private static final boolean invert;
  private static final boolean brake = false;
  private static final double reduction = 5.0 / 3.0;

  static {
    if (Constants.getRobot() == Constants.RobotType.COMPBOT) {
      id = 20;
      invert = true;
    } else {
      id = 20;
      invert = true;
    }
  }

  public IntakeIOKrakenFOC() {
    super(id, bus, currentLimitAmps, invert, brake, reduction);
  }
}
