// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import frc.robot.Constants;

public class ShooterConstants {

  // encoder / flywheelReduction = flywheel
  public static double reduction = (1.0 / 1.0);
  public static double shooterToleranceRPM = 100.0;
  public static int topID = 22;
  public static int bottomID = 23;

  public static Gains gains =
      switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(0.0, 0.0, 0.0, 0.09078, 0.00103, 0.0);
          // case COMPBOT -> new Gains(0.00009, 0.0000002, 0.05, 8.75, 0.0027, 0.0);
        case COMPBOT -> new Gains(0.000175, 0.0, 0.0, 0.122, 0.00175, 0.0);

          // case COMPBOT -> null;
      };

  public record Gains(double kP, double kI, double kD, double kS, double kV, double kA) {}
}
