// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeWheelsIO {
  @AutoLog
  public static class IntakeWheelsIOInputs {
    public double velocityRPM = 0.0;
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeWheelsIOInputs inputs) {}

  public default void setSpeedRPM(double velocityRPM) {}

  public default void setVotSpeed(double appliedVolts) {}

  public default void stop() {}
}
