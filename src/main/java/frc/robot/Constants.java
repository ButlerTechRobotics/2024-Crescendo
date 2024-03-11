// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.Alert;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int loopPeriodMs = 20;
  private static RobotType robotType = RobotType.COMPBOT;
  public static final boolean tuningMode = true;
  public static final boolean characterizationMode = false;

  public static RobotType getRobot() {
    if (RobotBase.isReal() && robotType == RobotType.SIMBOT) {
      new Alert("Invalid Robot Selected, using COMPBOT as default", Alert.AlertType.ERROR)
          .set(true);
      robotType = RobotType.COMPBOT;
    }
    return robotType;
  }

  public static Mode getMode() {
    return switch (getRobot()) {
      case COMPBOT -> RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
      case SIMBOT -> Mode.SIM;
    };
  }

  public enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum RobotType {
    SIMBOT,
    COMPBOT
  }

  /** Checks whether the robot the correct robot is selected when deploying. */
  public static void main(String... args) {
    if (robotType == RobotType.SIMBOT) {
      System.err.println("Cannot deploy, invalid robot selected: " + robotType.toString());
      System.exit(1);
    }
  }

  public static final class ControllerConstants {
    public static final double controllerDeadzone = 0.175;
  }

  public static final class MotorConstants {
    public static final int universalCurrentLimitAmps = 50;

    // Motor configs
    public static final int angleContinuousCurrentLimit = 50;
    public static final boolean angleInvert = true;
    public static final IdleMode angleNeutralMode = IdleMode.kCoast;

    public static final int driveContinuousCurrentLimit = 60;
    public static final boolean driveInvert = true;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;
  }

  public static final class SwerveModuleConstants {
    /** Rotations of the drive wheel per rotations of the drive motor. */
    public static final double driveGearReduction = (8.14 / 1.0);

    /** Rotations of the steering column per rotations of the angle motor. */
    public static final double steerGearReduction = (150.0 / 7.0);

    // The wheels have a 2 inch radius, but sink into the capet about (1/8) of an
    // inch
    // for an effective radius of 2-(1/8).
    public static final double wheelRadiusMeters = Units.inchesToMeters(2. - 1. / 8.);
    public static final double wheelCircumferenceMeters = 2 * Math.PI * wheelRadiusMeters;

    // PID + FEEDFORWARD CONSTANTS FOR MOTORS
    // PID for drive motors.
    public static final double drivekPVoltsPerMeterPerSecond = 0;
    public static final double drivekIVoltsPerMeter = 0.;
    public static final double drivekDVoltsPerMeterPerSecondSquared = 0.;

    // PID for angle motors.
    public static final double anglekPVoltsPerDegree = 0.08;
    public static final double anglekIVoltsPerDegreeSeconds =
        0.; // this might be the wrong unit idk
    public static final double anglekDVoltsPerDegreePerSecond = 0.;

    public static final double drivekSVolts = 0.;
    public static final double drivekVVoltsSecondsPerMeter =
        2.69; // 12.0/DrivetrainConstants.maxAchievableVelocityMetersPerSecond;
    // // TODO: this is a placeholder
    public static final double drivekAVoltsSecondsSquaredPerMeter = 0.;
  }

  public static final class GyroConstants {
    public static final int pigeonID = 30;

    // Follow the mount calibration process in Phoenix Tuner to determine these
    public static final double mountPoseYawDegrees = -0.118103;
    public static final double mountPosePitchDegrees = -0.703125;
    public static final double mountPoseRollDegrees = -0.043945;
  }

  public static final class DrivetrainConstants {
    // KINEMATICS CONSTANTS

    /**
     * Distance between the center point of the left wheels and the center point of the right
     * wheels.
     */
    public static final double trackwidthMeters = Units.inchesToMeters(23.75);

    /**
     * Distance between the center point of the front wheels and the center point of the back
     * wheels.
     */
    public static final double wheelbaseMeters = Units.inchesToMeters(22.75);

    /** Distance from the center of the robot to each swerve module. */
    public static final double drivetrainRadiusMeters =
        Math.hypot(wheelbaseMeters / 2.0, trackwidthMeters / 2.0);

    public static final SwerveDriveKinematics swerveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
            new Translation2d(wheelbaseMeters / 2.0, -trackwidthMeters / 2.0),
            new Translation2d(-wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
            new Translation2d(-wheelbaseMeters / 2.0, -trackwidthMeters / 2.0));

    /**
     * The maximum possible velocity of the robot in meters per second. <br>
     * This is a measure of how fast the robot will be able to drive in a straight line, based off
     * of the empirical free speed of the drive Krakens.
     */
    public static final double krakenFreeSpeedRPM = 6784;

    public static final double krakenFreeSpeedRotationsPerSecond = krakenFreeSpeedRPM / 60.;
    public static final double maxAchievableVelocityMetersPerSecond =
        krakenFreeSpeedRotationsPerSecond
            * SwerveModuleConstants.driveGearReduction
            * SwerveModuleConstants.wheelCircumferenceMeters;

    /**
     * This is the max desired speed that will be achievable in teleop. <br>
     * If the controller joystick is maxed in one direction, it will drive at this speed. <br>
     * This value will be less than or equal to the maxAchievableVelocityMetersPerSecond, depending
     * on driver preference.
     */
    public static final double maxDesiredTeleopVelocityMetersPerSecond =
        maxAchievableVelocityMetersPerSecond; // TODO:

    // placeholder

    /**
     * The maximum achievable angular velocity of the robot in radians per second. <br>
     * This is a measure of how fast the robot can rotate in place, based off of
     * maxAchievableVelocityMetersPerSecond.
     */
    public static final double maxAchievableAngularVelocityRadiansPerSecond =
        maxAchievableVelocityMetersPerSecond
            / Math.hypot(trackwidthMeters / 2.0, wheelbaseMeters / 2.0);

    /**
     * This is the max desired angular velocity that will be achievable in teleop. <br>
     * If the controller rotation joystick is maxed in one direction, it will rotate at this speed.
     * <br>
     * This value will be tuned based off of driver preference.
     */
    public static final double maxDesiredTeleopAngularVelocityRadiansPerSecond = 5.4;

    public static final double maxDesiredTeleopAccelMetersPerSecondSquared =
        100.0; // TODO: placeholder
  }

  public static final class VisionConstants {

    public static final AprilTagFieldLayout aprilTagFieldLayout =
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    public static final Transform3d robotToCamera =
        new Transform3d(
            new Translation3d(Units.inchesToMeters(8), 0, Units.inchesToMeters(11.5)),
            new Rotation3d(0, Math.toRadians(-28), 0));
  }

  public static final class FieldConstants {

    /** Distance from the front edge of the speaker structure to the carpet. */
    public static final double speakerHeightMeters = 2.09;

    /** X-Y position of the center of 30 cm behind the front edge of the red speaker. */
    public static final Translation2d blueSpeakerTranslation2d =
        new Translation2d(0.46 - 0.3, 5.55);

    /** X-Y position of the center of 30 cm behind the front edge of the blue speaker. */
    public static final Translation2d redSpeakerTranslation2d =
        new Translation2d(16.08 + 0.3, 5.55);

    /**
     * Distance from the floor to the center of the pivot. This is used for angle cal;culations for
     * shoot from anywhere.
     */
    public static final double pivotHeightMeters = Units.inchesToMeters(22);
  }
}
