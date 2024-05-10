// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.beambreak.BeamBreakHelper.BeamBreakValues;

public class BeamBreakIOSim implements BeamBreakIO {
  NetworkTable table;
  NetworkTableEntry magazineSensor;

  public BeamBreakIOSim() {
    table = NetworkTableInstance.getDefault().getTable("BeamBreak");
    magazineSensor = table.getEntry("MagazineSensor");
    magazineSensor.setBoolean(false);
  }

  @Override
  /** Shoots the game piece (empty magazines) */
  public void shootGamePiece() {
    magazineSensor.setBoolean(false);
  }

  @Override
  /** Sets the game piece sensors */
  public void setGamePiece(boolean magazine) {
    magazineSensor.setBoolean(magazine);
  }

  public void updateInputs(BeamBreakIOInputs inputs) {
    inputs.beamBreakValues = new BeamBreakValues(magazineSensor.getBoolean(false));
  }
}
