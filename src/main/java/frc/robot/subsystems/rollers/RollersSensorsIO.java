// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersSensorsIO {
  @AutoLog
  class RollersSensorsIOInputs {
    boolean shooterStaged = false;
    boolean backbackStaged = false;
  }

  default void updateInputs(RollersSensorsIOInputs inputs) {}
}
