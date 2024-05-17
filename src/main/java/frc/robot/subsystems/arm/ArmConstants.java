// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import frc.robot.Constants;

public class ArmConstants {

  // encoder / flywheelReduction = flywheel
  public static double reduction = (125.0 / 1.0);
  public static double armToleranceDeg = 1.0;
  public static int armID = 20;

  public static Gains gains =
      switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
          // case COMPBOT -> new Gains(0.00009, 0.0000002, 0.05, 8.75, 0.0027, 0.0);
        case COMPBOT -> new Gains(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0); // kG as .57

          // case COMPBOT -> null;
      };

  public record Gains(
      double kP,
      double kI,
      double kD,
      double kIz,
      double kFF,
      double kMinOutput,
      double kMaxOutput) {}
}
