// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.linebreak;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.linebreak.LineBreakHelper.LineBreakValues;

public class LineBreakIOSim implements LineBreakIO {
  NetworkTable table;
  NetworkTableEntry IntakeSensor;

  public LineBreakIOSim() {
    table = NetworkTableInstance.getDefault().getTable("LineBreak");
    IntakeSensor = table.getEntry("IntakeSensor");

    IntakeSensor.setBoolean(false);
  }

  @Override
  /** Shoots the game piece (empty magazines) */
  public void shootGamePiece() {
    IntakeSensor.setBoolean(false);
  }

  @Override
  /** Sets the game piece sensors */
  public void setGamePiece(boolean intake) {
    IntakeSensor.setBoolean(intake);
  }

  public void updateInputs(LineBreakIOInputs inputs) {
    inputs.lineBreakValues = new LineBreakValues(IntakeSensor.getBoolean(false));
  }
}
