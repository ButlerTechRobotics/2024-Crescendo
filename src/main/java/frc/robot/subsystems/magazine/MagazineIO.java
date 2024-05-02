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
    public double topVelocityRPM = 0.0;
    public double bottomVelocityRPM = 0.0;
    public double topAppliedVolts = 0.0;
    public double bottomAppliedVolts = 0.0;

    public double[] topCurrentAmps = new double[] {};
    public double[] bottomCurrentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(MagazineIOInputs inputs) {}

  public default void runVoltage(double topVoltage, double bottomVoltage) {}

  public default void stop() {}
}
