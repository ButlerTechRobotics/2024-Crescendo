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
  class ArmIOInputs {
    public double armPositionDeg = 0.0;
    public double armVelocityRpm = 0.0;
    public double armAppliedVolts = 0.0;
    public double armOutputCurrent = 0.0;
    public double armTempCelsius = 0.0;
  }

  /** Update inputs */
  default void updateInputs(ArmIOInputs inputs) {}

  /** Run both motors at voltage */
  default void runVolts(double armVolts) {}

  /** Stop both flywheels */
  public default void stop() {}

  /** Run top and bottom flywheels at velocity in rpm */
  default void runPosition(double angle) {}

  /** Config PID values for both motors */
  default void setPID(double kP, double kI, double kD) {}

  /** Config FF values for both motors */
  default void setFF(double kS, double kV, double kA) {}

  /** Run top flywheels at voltage */
  default void runCharacterizationArmVolts(double volts) {}
}
