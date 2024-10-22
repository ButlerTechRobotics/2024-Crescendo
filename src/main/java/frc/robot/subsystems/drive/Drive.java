// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;
import static frc.robot.subsystems.drive.DriveConstants.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SmartController;
import frc.robot.subsystems.drive.PoseEstimator.OdometryObservation;
import frc.robot.subsystems.drive.PoseEstimator.VisionObservation;
import frc.robot.util.LoggedTunableNumber;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private static final LoggedTunableNumber coastWaitTime = new LoggedTunableNumber("Drive/CoastWaitTimeSeconds", 0.5);
  private static final LoggedTunableNumber coastMetersPerSecThreshold = new LoggedTunableNumber(
      "Drive/CoastMetersPerSecThreshold", 0.05);

  public enum CoastRequest {
    AUTOMATIC,
    ALWAYS_BRAKE,
    ALWAYS_COAST
  }

  private final Timer lastMovementTimer = new Timer();

  @AutoLogOutput(key = "Drive/BrakeModeEnabled")
  private boolean brakeModeEnabled = true;

  @Setter
  @AutoLogOutput(key = "Drive/CoastRequest")
  private CoastRequest coastRequest = CoastRequest.AUTOMATIC;

  private boolean lastEnabled = false;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private double filteredX = 0;
  private double filteredY = 0;
  private final LinearFilter xFilter = LinearFilter.movingAverage(10);
  private final LinearFilter yFilter = LinearFilter.movingAverage(10);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(moduleTranslations);
  private Rotation2d rawGyroRotation = new Rotation2d();
  private double yawVelocityRadPerSec = 0;
  private double yawTimeStamp = 0;

  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition(),
          new SwerveModulePosition()
      };
  private final PoseEstimator poseEstimator;

  private static ProfiledPIDController thetaController = new ProfiledPIDController(
      headingControllerConstants.Kp(),
      0,
      headingControllerConstants.Kd(),
      new TrapezoidProfile.Constraints(
          drivetrainConfig.maxAngularVelocity(), drivetrainConfig.maxAngularAcceleration()));

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    System.out.println("[Init] Creating Drive");
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);

    // Start threads (no-op for each if no signals have been created)
    PhoenixOdometryThread.getInstance().start();

    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder
      AutoBuilder.configure(
          this::getPose,
          this::setPose,
          () -> kinematics.toChassisSpeeds(getModuleStates()),
          this::runVelocity,
          new PPHolonomicDriveController(
              new PIDConstants(
                  PPtranslationConstants.kP, PPtranslationConstants.kI, PPtranslationConstants.kD),
              new PIDConstants(
                  PProtationConstants.kP, PProtationConstants.kI, PProtationConstants.kD)),
          config,
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this);

    } catch (Exception e) {
      DriverStation.reportError(
          "Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
    }

    PathPlannerLogging.setLogActivePathCallback(
        activePath -> Logger.recordOutput(
            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()])));
    PathPlannerLogging.setLogTargetPoseCallback(
        targetPose -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose));

    poseEstimator = new PoseEstimator(DriveConstants.stateStdDevs, DriveConstants.kinematics);
    PPHolonomicDriveController.overrideRotationFeedback(this::getOverrideRotationFeedback);

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    thetaController.setTolerance(Units.degreesToRadians(1.5));
  }

  @Override
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    for (var module : modules) {
      module.updateInputs();
    }
    odometryLock.unlock();
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }
    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // Update odometry
    double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] = new SwerveModulePosition(
            modulePositions[moduleIndex].distanceMeters
                - lastModulePositions[moduleIndex].distanceMeters,
            modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
        yawVelocityRadPerSec = gyroInputs.yawVelocityRadPerSec;
        yawTimeStamp = gyroInputs.odometryYawTimestamps[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        yawTimeStamp = sampleTimestamps[i];
      }

      // Apply update
      poseEstimator.addOdometryObservation(
          new OdometryObservation(
              new SwerveDriveWheelPositions(modulePositions), rawGyroRotation, yawTimeStamp));

      ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
      Translation2d rawFieldRelativeVelocity = new Translation2d(chassisSpeeds.vxMetersPerSecond,
          chassisSpeeds.vyMetersPerSecond)
          .rotateBy(getRotation());

      filteredX = xFilter.calculate(rawFieldRelativeVelocity.getX());
      filteredY = yFilter.calculate(rawFieldRelativeVelocity.getY());
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    setModuleStates(setpointStates);
  }

  public void setModuleStates(SwerveModuleState[] setpointStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, drivetrainConfig.maxLinearVelocity());

    // Send setpoints to modules
    SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      // The module returns the optimized state, useful for logging
      optimizedSetpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Update brake mode
    // Reset movement timer if moved
    if (Arrays.stream(modules)
        .anyMatch(
            module -> Math.abs(module.getVelocityMetersPerSec()) > coastMetersPerSecThreshold.get())) {
      lastMovementTimer.reset();
    }
    if (DriverStation.isEnabled() && !lastEnabled) {
      coastRequest = CoastRequest.AUTOMATIC;
    }

    lastEnabled = DriverStation.isEnabled();
    switch (coastRequest) {
      case AUTOMATIC -> {
        if (DriverStation.isEnabled()) {
          setBrakeMode(true);
        } else if (lastMovementTimer.hasElapsed(coastWaitTime.get())) {
          setBrakeMode(false);
        }
      }
      case ALWAYS_BRAKE -> {
        setBrakeMode(true);
      }
      case ALWAYS_COAST -> {
        setBrakeMode(false);
      }
    }

    // Log setpoint states
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisSpeeds());
  }

  /**
   * Set brake mode to {@code enabled} doesn't change brake mode if already set.
   */
  private void setBrakeMode(boolean enabled) {
    if (brakeModeEnabled != enabled) {
      Arrays.stream(modules).forEach(module -> module.setBrakeMode(enabled));
    }
    brakeModeEnabled = enabled;
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement.
   * The modules will
   * return to their normal orientations the next time a nonzero velocity is
   * requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = moduleTranslations[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /**
   * Returns the module states (turn angles and drive velocities) for all of the
   * modules.
   */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /**
   * Returns the module positions (turn angles and drive positions) for all of the
   * modules.
   */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Runs forwards at the commanded voltage. */
  public void runCharacterizationVolts(double volts) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(volts);
    }
  }

  /** Returns the average drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    double driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getCharacterizationVelocity();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the current pose estimation. */
  @AutoLogOutput(key = "Odometry/PoseEstimation")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPose();
  }

  /** Returns the current poseEstimator rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /**
   * Resets the current poseEstimator pose.
   *
   * @param pose The pose to reset to.
   */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPose(pose);
  }

  /**
   * Adds vision data to the pose esimation.
   *
   * @param visionData The vision data to add.
   */
  public void addVisionData(List<VisionObservation> visionData) {
    visionData.stream()
        .sorted(Comparator.comparingDouble(VisionObservation::timestamp))
        .forEach(poseEstimator::addVisionObservation);
  }

  @AutoLogOutput
  public Translation2d getFieldRelativeVelocity() {
    return new Translation2d(filteredX, filteredY);
  }

  public static ProfiledPIDController getThetaController() {
    return thetaController;
  }

  public double getOverrideRotationFeedback() {
    // Some condition that should decide if we want to override rotation
    if (DriverStation.isAutonomous()
        && SmartController.getInstance().isSmartControlEnabled()
        && SmartController.getInstance().getDriveModeType() == SmartController.DriveModeType.SPEAKER) {
      // Return the rotation override (this should be a field relative rotation)
      return SmartController.getInstance().getTargetAimingParameters().robotAngle().getDegrees();
    } else {
      // return a default value when we don't want to override the path's rotation
      return 0.0; // or any default value
    }
  }

  public double[] getWheelRadiusCharacterizationPosition() {
    return Arrays.stream(modules).mapToDouble(Module::getPositionRadians).toArray();
  }

  public void setWheelsToCircle() {
    Rotation2d[] turnAngles = Arrays.stream(DriveConstants.moduleTranslations)
        .map(translation -> translation.getAngle().plus(new Rotation2d(Math.PI / 2.0)))
        .toArray(Rotation2d[]::new);
    SwerveModuleState[] desiredStates = Arrays.stream(turnAngles)
        .map(angle -> new SwerveModuleState(0, angle))
        .toArray(SwerveModuleState[]::new);
    setModuleStates(desiredStates);
  }
}
