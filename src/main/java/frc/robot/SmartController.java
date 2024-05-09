// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.FieldConstants;
import org.littletonrobotics.junction.Logger;

/**
 * The SmartController class represents a controller for the robot's system. It provides methods to
 * control the robot.
 *
 * <p>Most of the methods in this class handle state machine logic, which is a way to represent the
 * state of the robot and the transitions between states.
 *
 * <p>The SmartController class is a singleton, which means that there is only one instance of the
 * class that is shared across the entire robot code.
 */
public class SmartController {
  private static SmartController instance;

  private DriveModeType driveModeType = DriveModeType.SAFE;
  private AimingParameters targetAimingParameters =
      new AimingParameters(Rotation2d.fromDegrees(90), 0.0, 40.5, 0, 0);

  // Whether or not the robot is in smart control mode. Smart control mode is a
  // mode where the robot
  // automatically adjusts its heading to face a target.
  private boolean smartControl = false;
  private boolean emergencyIntakeMode = false;

  // Distance to the target when we should start running flywheel if we have a
  // game piece
  private double prerollDistance = 8.002;

  // Interpolation maps for shooting into the speaker
  private final InterpolatingDoubleTreeMap shooterSpeedMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap armAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap flightTimeMap = new InterpolatingDoubleTreeMap();

  // Interpolation maps for feeding shots (passing from mid field to amp area)
  private final InterpolatingDoubleTreeMap feederSpeedMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap feederAngleMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap feederFlightTimeMap = new InterpolatingDoubleTreeMap();

  private SmartController() {

    // Units: RPM
    shooterSpeedMap.put(1.0, 3000.0);
    shooterSpeedMap.put(1.5, 3000.0);
    shooterSpeedMap.put(2.0, 3200.0);
    shooterSpeedMap.put(2.5, 3500.0);
    shooterSpeedMap.put(3.0, 3800.0);
    shooterSpeedMap.put(3.5, 4250.0); // 4000 4/4/2024 3:56PM
    shooterSpeedMap.put(3.75, 4250.0); // 4000 4/4/2024 3:56PM
    shooterSpeedMap.put(4.0, 4400.0); // 4200 4/4/2024 3:56PM
    shooterSpeedMap.put(4.2, 4400.0); // 4250 4/4/2024 3:56PM
    shooterSpeedMap.put(4.5, 4400.0); // 4300 4/4/2024 3:56PM
    shooterSpeedMap.put(5.0, 4400.0);
    shooterSpeedMap.put(5.5, 4500.0);
    shooterSpeedMap.put(6.0, 4750.0);
    shooterSpeedMap.put(6.5, 5000.0);
    shooterSpeedMap.put(7.0, 5500.0);
    shooterSpeedMap.put(8.0, 6500.0);

    // // Units: Degress
    armAngleMap.put(1.0, 0.0);
    armAngleMap.put(1.5, 7.88 + 1.25);
    armAngleMap.put(2.0, 12.02 + 1.25);
    armAngleMap.put(2.25, 15.0 + 1.25);
    armAngleMap.put(2.5, 19.7 + 1.25); // /20.0
    armAngleMap.put(3.0, 23.1 + 1.25);
    armAngleMap.put(3.5, 26.60 + 1.25); // 26.9
    armAngleMap.put(3.75, 27.18 + 1.25); // 27.5
    armAngleMap.put(4.0, 27.8 + 1.25); // 28.02
    armAngleMap.put(4.5, 30.20 + 1.25); // 30.75
    armAngleMap.put(5.0, 31.10 + 1.25); // 31.5
    armAngleMap.put(5.5, 32.75 + 1.25);
    armAngleMap.put(6.0, 33.50 + 1.25);
    armAngleMap.put(6.5, 35.0 + 1.25);
    armAngleMap.put(7.0, 35.5 + 1.25);
    armAngleMap.put(8.0, 36.75 + 1.25);
    armAngleMap.put(9.0, 38.75 + 1.25);
    armAngleMap.put(9.5, 39.75 + 1.25);

    flightTimeMap.put(5.0, 0.35);
    flightTimeMap.put(4.0, 0.31);
    flightTimeMap.put(3.0, 0.31);
    flightTimeMap.put(2.0, 0.25);
    flightTimeMap.put(1.0, 0.15);

    // Feed Maps
    feederSpeedMap.put(9.071, 3500.0);
    feederSpeedMap.put(7.0, 3000.0);
    feederSpeedMap.put(5.4, 2000.0);

    feederAngleMap.put(10.0, 5.0);
    feederAngleMap.put(9.0, 3.0);
    feederAngleMap.put(8.0, 0.0);

    feederFlightTimeMap.put(30.0, 3.0); // Way further than we should ever be shooting
    feederFlightTimeMap.put(9.59, 1.0); // 1.9
    feederFlightTimeMap.put(7.2, 0.9); // 1.68
    feederFlightTimeMap.put(5.4, 0.8); // 1.36
    feederFlightTimeMap.put(0.0, 0.0); // Way less than we should ever be shooting
  }

  /**
   * Gets the instance of the SmartController.
   *
   * @return The instance of the SmartController.
   */
  public static SmartController getInstance() {
    if (instance == null) {
      instance = new SmartController();
    }
    Logger.recordOutput("SmartController/smartControl", instance.smartControl);
    Logger.recordOutput("SmartController/driveModeType", instance.driveModeType.toString());
    return instance;
  }

  /**
   * Gets the shooter speed for a given distance.
   *
   * @param distance The distance to the target in meters.
   * @return The shooter speed for the given distance.
   */
  public Double getShooterSpeed(double distance) {
    return shooterSpeedMap.get(distance);
  }

  /**
   * Gets the shooter angle for a given distance.
   *
   * @param distance The distance to the target in inches.
   * @return The shooter angle for the given distance.
   */
  public Double getArmAngle(double distance) {
    return armAngleMap.get(Units.inchesToMeters(distance));
  }

  /**
   * Checks if the heading is being controlled.
   *
   * @return True if the heading is being controlled, false otherwise.
   */
  public boolean isSmartControlEnabled() {
    return this.smartControl;
  }

  /**
   * Sets the emergency intake mode. This is used if the normal intake is not working properly.
   * Emergency intake mode allows us to pick up directly from the source using the shooter.
   *
   * @param emergencyMode The emergency mode to set.
   */
  public void setEmergencyIntakeMode(boolean emergencyMode) {
    this.emergencyIntakeMode = emergencyMode;
  }

  /**
   * Gets the emergency intake mode.
   *
   * @return True if the emergency intake mode is enabled, false otherwise.
   */
  public boolean getEmergencyIntakeMode() {
    return this.emergencyIntakeMode;
  }

  /** Toggles the emergency intake mode. */
  public void toggleEmergencyIntakeMode() {
    this.emergencyIntakeMode = !emergencyIntakeMode;
  }

  /**
   * Gets the current drive mode.
   *
   * @return The supplier that provides the current drive mode.
   */
  public DriveModeType getDriveModeType() {
    return this.driveModeType;
  }

  /**
   * Sets the drive mode.
   *
   * @param driveModeType The drive mode to set.
   */
  public void setDriveMode(DriveModeType driveModeType) {
    this.driveModeType = driveModeType;
  }

  /** Enables heading control based on the current drive mode. */
  public void enableSmartControl() {
    this.smartControl = true;
  }

  /** Disables heading control (heading control is disabled). */
  public void disableSmartControl() {
    this.smartControl = false;
  }

  /**
   * Calculates the shooter parameters for a given field relative pose, velocity, and acceleration.
   *
   * <p>This handles calculating target robot angle, radial velocity, shooter speed, shooter angle.
   *
   * <p>It also is done in a way that if the robot is moving, it will adjust the location of the
   * target to account for the robot's movement so the shot will still hit.
   *
   * @param fieldRelativePose The pose of the robot on the field
   * @param fieldRelativeVelocity The velocity of the robot on the field
   * @param fieldRelativeAcceleration The acceleration of the robot on the field
   */
  public void calculateSpeaker(
      Pose2d fieldRelativePose,
      Translation2d fieldRelativeVelocity,
      Translation2d fieldRelativeAcceleration) {
    Logger.recordOutput("ShotCalculator/fieldRelativePose", fieldRelativePose);
    Logger.recordOutput("ShotCalculator/fieldRelativeVelocity", fieldRelativeVelocity);
    Logger.recordOutput("ShotCalculator/fieldRelativeAcceleration", fieldRelativeAcceleration);

    setPrerollDistance(8.002);

    Translation2d speakerPose =
        AllianceFlipUtil.apply(FieldConstants.Speaker.centerSpeakerOpening.getTranslation());

    // Calculate the distance to the speaker and the time it will take to get there
    double distanceToSpeaker = fieldRelativePose.getTranslation().getDistance(speakerPose);
    double shotTime = flightTimeMap.get(distanceToSpeaker);

    // Calculate the new location of the speaker based on the robot's movement
    Translation2d speedAccComp = fieldRelativeVelocity.plus(fieldRelativeAcceleration.times(0.025));
    Translation2d movingGoalLocation = speakerPose.minus(speedAccComp.times(shotTime));
    Translation2d toTestGoal = movingGoalLocation.minus(fieldRelativePose.getTranslation());

    // Calculate the new time it will take to get to the new location of the speaker
    double effectiveDistanceToSpeaker = toTestGoal.getNorm();
    double newShotTime = flightTimeMap.get(effectiveDistanceToSpeaker);

    // Iterate through the calculation a few times to get a more accurate shot time
    for (int i = 0; i < 3 && Math.abs(newShotTime - shotTime) > 0.01; i++) {
      shotTime = newShotTime;
      speedAccComp = fieldRelativeVelocity.plus(fieldRelativeAcceleration.times(0.025));
      movingGoalLocation = speakerPose.minus(speedAccComp.times(shotTime));
      toTestGoal = movingGoalLocation.minus(fieldRelativePose.getTranslation());
      effectiveDistanceToSpeaker = toTestGoal.getNorm();
      newShotTime = flightTimeMap.get(effectiveDistanceToSpeaker);
    }

    // Set the target angle of the robot to point at the target
    Rotation2d setpointAngle =
        movingGoalLocation
            .minus(fieldRelativePose.getTranslation())
            .getAngle()
            .plus(Rotation2d.fromDegrees(180));
    double angleDifference = setpointAngle.minus(fieldRelativePose.getRotation()).getRadians();

    // Assuming a constant linear velocity (you can adjust this)
    // double assumedLinearVelocity = fieldRelativeVelocity.getNorm();

    // Calculate tangential velocity using linear velocity and angle difference

    // double tangentialVelocity = assumedLinearVelocity *
    // Math.sin(angleDifference);

    // Now, calculate angular velocity using tangential velocity and
    // newDistanceToSpeaker

    // double radialVelocity = tangentialVelocity / newDistanceToSpeaker;
    double radialVelocity = 0.0;
    // Log the outputs
    Logger.recordOutput("ShotCalculator/effectiveDistanceToSpeaker", effectiveDistanceToSpeaker);
    Logger.recordOutput(
        "ShotCalculator/effectiveAimingPose", new Pose2d(movingGoalLocation, setpointAngle));
    Logger.recordOutput("ShotCalculator/angleDifference", angleDifference);
    Logger.recordOutput("ShotCalculator/radialVelocity", radialVelocity);

    // Set the target aiming parameters
    setTargetAimingParameters(
        new AimingParameters(
            setpointAngle,
            radialVelocity,
            shooterSpeedMap.get(effectiveDistanceToSpeaker),
            armAngleMap.get(effectiveDistanceToSpeaker),
            effectiveDistanceToSpeaker));
  }

  /**
   * Calculates the shooter parameters for the amp shot.
   *
   * <p>This will always be static and will not adjust for the robot's movement.
   */
  public void calculateAmp() {
    setTargetAimingParameters(new AimingParameters(Rotation2d.fromDegrees(90), 0.0, 20, 0, 0));
  }

  /**
   * Calculates the shooter parameters for the feeding shot.
   *
   * <p>This is identical to the speaker shot, but the target is the feed location next to the amp.
   */
  public void calculateFeed(Pose2d fieldRelativePose, Translation2d fieldRelativeVelocity) {
    setPrerollDistance(FieldConstants.fieldLength);
    Translation2d feedLocation = AllianceFlipUtil.apply(FieldConstants.cornerFeedLocation);
    double distanceToFeedLocation = fieldRelativePose.getTranslation().getDistance(feedLocation);
    double shotTime = feederFlightTimeMap.get(distanceToFeedLocation);
    Translation2d movingGoalLocation = feedLocation.minus(fieldRelativeVelocity.times(shotTime));
    Translation2d toTestGoal = movingGoalLocation.minus(fieldRelativePose.getTranslation());
    double effectiveDistanceToFeedLocation = toTestGoal.getNorm();
    Rotation2d setpointAngle =
        movingGoalLocation
            .minus(fieldRelativePose.getTranslation())
            .getAngle()
            .plus(Rotation2d.fromDegrees(180));
    double angleDifference = setpointAngle.minus(fieldRelativePose.getRotation()).getRadians();
    double radialVelocity = 0.0;
    Logger.recordOutput(
        "ShotCalculator/effectiveDistanceToFeedLocation", effectiveDistanceToFeedLocation);
    Logger.recordOutput(
        "ShotCalculator/effectiveAimingPose", new Pose2d(movingGoalLocation, setpointAngle));
    Logger.recordOutput("ShotCalculator/angleDifference", angleDifference);
    Logger.recordOutput("ShotCalculator/radialVelocity", radialVelocity);

    setTargetAimingParameters(
        new AimingParameters(
            setpointAngle,
            radialVelocity,
            feederSpeedMap.get(effectiveDistanceToFeedLocation),
            feederAngleMap.get(effectiveDistanceToFeedLocation),
            effectiveDistanceToFeedLocation));
  }

  /**
   * Sets the target aiming parameters.
   *
   * @param targetAimingParameters The target aiming parameters to set.
   */
  private void setTargetAimingParameters(AimingParameters targetAimingParameters) {
    this.targetAimingParameters = targetAimingParameters;
  }

  /**
   * Gets the target aiming parameters.
   *
   * <p>The target aiming parameters are the information that the robot should use to hit the
   * target.
   *
   * @return The target aiming parameters.
   */
  public AimingParameters getTargetAimingParameters() {
    return targetAimingParameters;
  }

  /**
   * Gets the distance to the target when we should start running the flywheel if we have a game
   * piece.
   *
   * @return The preroll distance.
   */
  public double getPrerollDistance() {
    return prerollDistance;
  }

  /**
   * Sets the distance to the target when we should start running the flywheel if we have a game
   * piece.
   *
   * @param prerollDistance The preroll distance to set.
   */
  private void setPrerollDistance(double prerollDistance) {
    this.prerollDistance = prerollDistance;
  }

  public record AimingParameters(
      Rotation2d robotAngle,
      double radialVelocity,
      double shooterSpeed,
      double armAngle,
      double effectiveDistanceToTarget) {}

  /** Possible Drive Modes. */
  public enum DriveModeType {
    AMP,
    SPEAKER,
    SAFE,
    FEED
  }
}
