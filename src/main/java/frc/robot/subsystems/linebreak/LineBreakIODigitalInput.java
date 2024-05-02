// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.linebreak;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.linebreak.LineBreakHelper.LineBreakValues;

public class LineBreakIODigitalInput implements LineBreakIO {
  DigitalInput IntakeSensor = new DigitalInput(7);

  public void updateInputs(LineBreakIOInputs inputs) {
    inputs.lineBreakValues = new LineBreakValues(!IntakeSensor.get());
  }
}
