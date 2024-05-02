// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double armAbsolutePositionDeg = 0.0;
    public double armRelativePositionDeg = 0.0;
    public double armVelocityRadPerSec = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public double[] armTempCelcius = new double[] {};
  }

  public default void setArmTarget(double target) {}

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setBrakeMode(boolean armBrake) {}

  public default void stop() {}
}
