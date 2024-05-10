// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.beambreak;

public class BeamBreakHelper {
  public record BeamBreakValues(boolean magazine) {
    // Gets the value of the digital input.  Returns true if the circuit is open.
    // True = game piece

    public boolean hasGamePiece() {
      return magazine;
    }

    public boolean notInMagazine() {
      return !(magazine);
    }

    public boolean inMagazine() {
      return (magazine);
    }

    public boolean isShooterLoaded() {
      return (magazine);
    }
  }
}
