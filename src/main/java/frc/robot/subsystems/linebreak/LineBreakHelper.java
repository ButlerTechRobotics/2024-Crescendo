// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.linebreak;

public class LineBreakHelper {
  public record LineBreakValues(boolean Intake) {
    // Gets the value of the digital input.  Returns true if the circuit is open.
    // True = game piece

    public boolean hasGamePiece() {
      return Intake;
    }

    public boolean notInIntake() {
      return !(Intake);
    }

    public boolean inIntake() {
      return (Intake);
    }

    public boolean hasGamePieceIntake() {
      return Intake;
    }

    public boolean isShooterLoaded() {
      return (Intake);
    }
  }
}
