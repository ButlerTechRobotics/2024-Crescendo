// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.beambreak.BeamBreakHelper.BeamBreakValues;

public class BeamBreakIODigitalInput implements BeamBreakIO {
  DigitalInput magazine = new DigitalInput(7);

  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.beamBreakValues = new BeamBreakValues(!magazine.get());
  }
}
