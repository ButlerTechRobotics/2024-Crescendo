// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import com.pathplanner.lib.config.PIDConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
public final class DriveConstants {
  public static DrivetrainConfig drivetrainConfig =
      switch (Constants.getRobot()) {
        default ->
            new DrivetrainConfig(
                Units.inchesToMeters(1.8799), // double wheelRadius
                Units.inchesToMeters(22.0), // double trackwidthX
                Units.inchesToMeters(22.0), // double trackwidthY
                Units.feetToMeters(16.055), // double maxLinearVelocity
                Units.feetToMeters(28.718), // double maxLinearAcceleration
                12.384, // double maxAngularVelocity
                31.319); // double maxAngularAcceleration)
      };

  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(
            drivetrainConfig.trackwidthX() / 2.0, drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            drivetrainConfig.trackwidthX() / 2.0, -drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            -drivetrainConfig.trackwidthX() / 2.0, drivetrainConfig.trackwidthY() / 2.0),
        new Translation2d(
            -drivetrainConfig.trackwidthX() / 2.0, -drivetrainConfig.trackwidthY() / 2.0)
      };
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);

  public static final double odometryFrequency =
      switch (Constants.getRobot()) {
        case SIMBOT -> 50.0;
        case COMPBOT -> 50.0;
      };
  public static Matrix<N3, N1> stateStdDevs =
      switch (Constants.getRobot()) {
        default -> new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002));
      };

  public static void updateStateStdDevs(double n1, double n2, double n3) {
    stateStdDevs = new Matrix<>(VecBuilder.fill(n1, n2, n3));
  }

  public static final double xyStdDevCoefficient =
      switch (Constants.getRobot()) {
        default -> 0.01;
      };

  public static final double thetaStdDevCoefficient =
      switch (Constants.getRobot()) {
        default -> 0.01;
      };

  public static final int gyroID = 30;

  // Turn to "" for no canbus name
  public static final String canbus = "rio";

  public static ModuleConfig[] moduleConfigs =
      switch (Constants.getRobot()) {
        case COMPBOT ->
            new ModuleConfig[] {
              new ModuleConfig(
                  1,
                  2,
                  9,
                  Rotation2d.fromRotations(0.097412).plus(Rotation2d.fromDegrees(0)),
                  true),
              new ModuleConfig(
                  3,
                  4,
                  10,
                  Rotation2d.fromRotations(0.020508).plus(Rotation2d.fromDegrees(0)),
                  true),
              new ModuleConfig(
                  5,
                  6,
                  11,
                  Rotation2d.fromRotations(-0.219238).plus(Rotation2d.fromDegrees(0)),
                  true),
              new ModuleConfig(
                  7,
                  8,
                  12,
                  Rotation2d.fromRotations(-0.429932).plus(Rotation2d.fromDegrees(180)),
                  true)
            };
          // .plus(Rotation2d.fromDegrees(180))
        case SIMBOT -> {
          ModuleConfig[] configs = new ModuleConfig[4];
          for (int i = 0; i < configs.length; i++)
            configs[i] = new ModuleConfig(0, 0, 0, new Rotation2d(0), false);
          yield configs;
        }
      };

  public static final ModuleConstants moduleConstants =
      switch (Constants.getRobot()) {
        case COMPBOT ->
            new ModuleConstants(
                0.1,
                0.13,
                0.1,
                0.0,
                10.0,
                0.0,
                Mk4iReductions.L2PLUS.reduction,
                Mk4iReductions.TURN.reduction);
        case SIMBOT ->
            new ModuleConstants(
                0.014,
                0.134,
                0.1,
                0.0,
                10.0,
                0.0,
                Mk4iReductions.L2PLUS.reduction,
                Mk4iReductions.TURN.reduction);
      };

  public static HeadingControllerConstants headingControllerConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> new HeadingControllerConstants(3.0, .125);
        case SIMBOT -> new HeadingControllerConstants(7, 0.125);
      };

  // ================================================
  // Pathplanner Config
  // ================================================
  public static final PIDConstants PPtranslationConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> new PIDConstants(3, 0.0, 0.0);
        case SIMBOT -> new PIDConstants(10, 0.0, 0.0);
      };

  public static final PIDConstants PProtationConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> new PIDConstants(3, 0.0, 0.0);
        case SIMBOT -> new PIDConstants(10, 0.0, 0.0);
      };

  public static PPModuleConfig ppModuleConfig =
      switch (Constants.getRobot()) {
        default ->
            new PPModuleConfig(
                Units.inchesToMeters(drivetrainConfig.wheelRadius), // double wheelRadiusMeters
                Units.feetToMeters(17.88058), // double maxDriveVelocityMPS
                1.000, // double wheelCOF
                DCMotor.getNeoVortex(1), // DCMotor driveMotor
                60, // double driveCurrentLimit
                1); // int numMotors
      };

  public static PPRobotConfig ppRobotConfig =
      switch (Constants.getRobot()) {
        default ->
            new PPRobotConfig(
                Units.lbsToKilograms(120), // double massKG
                6.883, // double MOI
                ppModuleConfig, // PPModuleConfig ppModuleConfig
                Units.inchesToMeters(drivetrainConfig.trackwidthX), // double trackwidthMeters
                Units.inchesToMeters(drivetrainConfig.trackwidthY)); // double wheelbaseMeters
      };

  public record DrivetrainConfig(
      double wheelRadius,
      double trackwidthX,
      double trackwidthY,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    public double driveBaseRadius() {
      return Math.hypot(trackwidthX / 2.0, trackwidthY / 2.0);
    }
  }

  public record ModuleConfig(
      int driveID,
      int turnID,
      int absoluteEncoderChannel,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted) {}

  public record ModuleConstants(
      double ffKs,
      double ffKv,
      double driveKp,
      double drivekD,
      double turnKp,
      double turnkD,
      double driveReduction,
      double turnReduction) {}

  public record PPModuleConfig(
      double wheelRadiusMeters,
      double maxDriveVelocityMPS,
      double wheelCOF,
      DCMotor driveMotor,
      double driveCurrentLimit,
      int numMotors) {}

  public record PPRobotConfig(
      double massKG,
      double MOI,
      PPModuleConfig ppModuleConfig,
      double trackwidthMeters,
      double wheelbaseMeters) {}

  public record HeadingControllerConstants(double Kp, double Kd) {}

  private enum Mk4iReductions {
    L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)),
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L2PLUS((50.0 / 16.0) * (17.0 / 27.0) * (45.0 / 15.0)),
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)),
    TURN((150.0 / 7.0)); // MK4i
    // TURN((12.8 / 1.0)); //MK4

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
