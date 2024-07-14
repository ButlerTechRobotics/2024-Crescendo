// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.gamepieceVision;

import org.littletonrobotics.junction.AutoLog;

public interface GamepieceVisionIO {
  @AutoLog
  class GamepieceVisionIOInputs {
    public boolean hasTarget = false;
    public double tx = 0.0;
    public double ty = 0.0;
    public double timestamp = 0.0;
    public double latency = 0.0;
    public double lastFPSTimestamp = 0.0;
  }

  void updateInputs(GamepieceVisionIOInputs inputs);
}
