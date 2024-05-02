// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.linebreak;

import frc.robot.subsystems.linebreak.LineBreakHelper.LineBreakValues;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface LineBreakIO {
  class LineBreakIOInputs implements LoggableInputs {

    LineBreakValues lineBreakValues = new LineBreakValues(false);

    @Override
    public void toLog(LogTable table) {
      table.put("LineBreaker/Intake", lineBreakValues.Intake());
    }

    @Override
    public void fromLog(LogTable table) {
      table.get("LineBreaker/Intake", lineBreakValues.Intake());
    }
  }

  public default void shootGamePiece() {}

  public default void setGamePiece(boolean Intake) {}

  default void updateInputs(LineBreakIOInputs inputs) {}
}
