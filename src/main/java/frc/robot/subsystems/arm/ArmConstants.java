// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public final class ArmConstants {
  public static final ArmPositions amp = new ArmPositions(Rotation2d.fromDegrees(80));
  public static final ArmPositions intake = new ArmPositions(Rotation2d.fromDegrees(0));
  public static final ArmPositions shoot = new ArmPositions(Rotation2d.fromDegrees(0));
  //   public static final ArmPositions trap =
  //       new ArmPositions(Rotation2d.fromDegrees(32), Rotation2d.fromDegrees(34));
  public static final ArmPositions feed = new ArmPositions(Rotation2d.fromDegrees(0));

  //   public static final ArmPositions manualShot =
  //       new ArmPositions(Rotation2d.fromDegrees(-23), Rotation2d.fromDegrees(78.5));
  public static final ArmPositions manualShot = new ArmPositions(Rotation2d.fromDegrees(0));

  public static final MotorFeedbackController armControlConstants =
      switch (Constants.getRobot()) {
        case SIMBOT -> new MotorFeedbackController(0, 0, 0, 0);
        case COMPBOT -> new MotorFeedbackController(0.055, 0.0001, 0.0, 0.000);
          // case COMPBOT -> new MotorFeedbackController(0, 0, 0, 0);
      };

  public static final int ARM_GEAR_RATIO = 125;

  public record ArmPositions(Rotation2d arm) {}

  public record MotorFeedbackController(double kP, double kI, double kD, double kG) {}
}
