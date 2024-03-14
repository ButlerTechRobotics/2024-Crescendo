// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ArmConstants {
  // reduction is 12:62 18:60 12:65
  public static final double reduction = (80.0 / 1.0);
  public static final Rotation2d positionTolerance = Rotation2d.fromDegrees(3.0);
  public static final Translation2d armOrigin = new Translation2d(-0.238, 0.298);
  public static final Rotation2d minAngle =
      switch (Constants.getRobot()) {
        default -> Rotation2d.fromDegrees(6.85); // Measured from hardstop 3/12/24
        case COMPBOT -> Rotation2d.fromDegrees(10.0);
      };
  public static final Rotation2d maxAngle = Rotation2d.fromDegrees(100.0);

  public static final int leaderID =
      switch (Constants.getRobot()) {
        default -> 11;
        case COMPBOT -> 25;
      };
  public static final int followerID =
      switch (Constants.getRobot()) {
        default -> 10;
        case COMPBOT -> 26;
      };
  public static final int armEncoderID =
      switch (Constants.getRobot()) {
        default -> 0;
        case COMPBOT -> 42;
      };

  public static final boolean leaderInverted = false;

  /**
   * Minimum angle of the arm, in degrees. This value should be negative, as it is below the
   * horizontal.
   */
  public static final double armMinAngleDegrees = 0.9;

  /**
   * Maximum angle of the arm, in degrees. This value should be positive and greater than 90, as it
   * is beyond the vertical.
   */
  public static final double armMaxAngleDegrees = 100.;

  /** The offset of the arm encoder in radians. */
  public static final double armEncoderOffsetRads =
      switch (Constants.getRobot()) {
        default -> 1.1980389953386859; // 1.0753205323078345 rad as measured at hardstop on 3/12/24,
          // corresponding to an arm position of 0.11965050145508001 rad
        case COMPBOT -> -1.233 - Math.PI / 2.0;
      };

  public static final double armLength =
      switch (Constants.getRobot()) {
        case COMPBOT -> Units.inchesToMeters(29.6);
        default -> Units.inchesToMeters(29.6);
      };

  public static final Gains gains =
      switch (Constants.getRobot()) {
        case SIMBOT -> new Gains(90.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        case COMPBOT -> new Gains(6000.0, 0.0, 250.0, 8.4, 0.0, 0.0, 22.9);
      };

  /***** REAL CONSTANTS ******/
  public static final double kSArmVolts = 0.005;

  public static final double kGArmVolts = 0.32;
  public static final double kVArmVoltsSecondsPerRadian = 3.1;
  public static final double kAArmVoltsSecondsSquaredPerRadian = 0;

  public static final double kPArmVoltsPerDegree = 0.3;
  public static final double kDArmVoltsSecondsPerDegree = 0.005;

  public static TrapezoidProfile.Constraints profileConstraints =
      new TrapezoidProfile.Constraints(2 * Math.PI, 15);

  public record Gains(
      double kP, double kI, double kD, double ffkS, double ffkV, double ffkA, double ffkG) {}
}
