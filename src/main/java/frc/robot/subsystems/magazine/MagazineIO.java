// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.magazine;

import org.littletonrobotics.junction.AutoLog;

public interface MagazineIO {
  @AutoLog
  public static class MagazineIOInputs {
    public double frontMotorVelocityRadPerSec = 0.0;
    public double frontMotorAppliedVolts = 0.0;
    public double frontMotorCurrentAmps = 0.0;

    public double backMotorVelocityRadPerSec = 0.0;
    public double backMotorAppliedVolts = 0.0;
    public double backMotorCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(MagazineIOInputs inputs) {}

  public default void runVoltage(double frontMotorVoltage, double backMotorVoltage) {}

  public default void stop() {}
}
