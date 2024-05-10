// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.beambreak;

import frc.robot.subsystems.beambreak.BeamBreakHelper.BeamBreakValues;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface BeamBreakIO {
  class BeamBreakIOInputs implements LoggableInputs {

    BeamBreakValues beamBreakValues = new BeamBreakValues(false);

    @Override
    public void toLog(LogTable table) {
      table.put("BeamBreaker/magazine", beamBreakValues.magazine());
    }

    @Override
    public void fromLog(LogTable table) {
      table.get("BeamBreaker/magazine", beamBreakValues.magazine());
    }
  }

  public default void shootGamePiece() {}

  public default void setGamePiece(boolean magazine) {}

  default void updateInputs(BeamBreakIOInputs inputs) {}
}
