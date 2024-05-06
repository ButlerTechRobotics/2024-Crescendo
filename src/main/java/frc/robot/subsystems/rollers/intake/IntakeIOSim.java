// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.subsystems.rollers.GenericRollerSystemIOSim;

public class IntakeIOSim extends GenericRollerSystemIOSim implements IntakeIO {
  private static final DCMotor motorModel = DCMotor.getNeoVortex(1);
  private static final double reduction = (18.0 / 12.0);
  private static final double moi = 0.001;

  public IntakeIOSim() {
    super(motorModel, reduction, moi);
  }
}
