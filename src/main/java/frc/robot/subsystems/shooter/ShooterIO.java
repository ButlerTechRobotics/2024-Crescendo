// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double topPositionRads = 0.0;
    public double topVelocityRPM = 0.0 * -1.;
    public double topAppliedVolts = 0.0;
    public double topOutputCurrent = 0.0;
    public double topTempCelsius = 0.0;

    public double bottomPositionRads = 0.0;
    public double bottomVelocityRPM = 0.0 * -1.;
    public double bottomAppliedVolts = 0.0;
    public double bottomOutputCurrent = 0.0;
    public double bottomTempCelsius = 0.0;
  }

  /** Update inputs */
  default void updateInputs(ShooterIOInputs inputs) {}

  /** Run both motors at voltage */
  default void runVolts(double topVolts, double bottomVolts) {}

  /** Stop both flywheels */
  public default void stop() {}

  /** Run top and bottom flywheels at velocity in rpm */
  default void runVelocity(double topRpm, double bottomRpm) {}

  /** Config PID values for both motors */
  default void setPID(double kP, double kI, double kD) {}

  /** Config FF values for both motors */
  default void setFF(double kS, double kV, double kA) {}

  /** Run top flywheels at voltage */
  default void runCharacterizationTopVolts(double volts) {}

  /** Run bottom flywheels at voltage */
  default void runCharacterizationBottomVolts(double volts) {}
}
