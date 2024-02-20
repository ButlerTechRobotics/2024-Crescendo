package frc.robot.subsystems.drive;

// Import necessary libraries
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

/** All Constants Measured in Meters and Radians (m/s, m/s^2, rad/s, rad/s^2) */
public final class DriveConstants {
  // Define the drivetrain configuration based on the robot type
  public static DrivetrainConfig drivetrainConfig =
      switch (Constants.getRobot()) {
        default -> // Default drivetrain configuration
            new DrivetrainConfig(
                Units.inchesToMeters(2.0), // Wheel radius
                Units.inchesToMeters(26.0), // Trackwidth X
                Units.inchesToMeters(26.0), // Trackwidth Y
                Units.feetToMeters(12.16), // Max linear velocity
                Units.feetToMeters(21.32), // Max linear acceleration
                7.93, // Max angular velocity
                29.89); // Max angular acceleration
      };
  // Define the wheel radius
  public static final double wheelRadius = Units.inchesToMeters(2.0);
  // Define the module translations for the swerve drive modules
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(
            drivetrainConfig.trackwidthX() / 2.0,
            drivetrainConfig.trackwidthY() / 2.0), // Front right module
        new Translation2d(
            drivetrainConfig.trackwidthX() / 2.0,
            -drivetrainConfig.trackwidthY() / 2.0), // Front left module
        new Translation2d(
            -drivetrainConfig.trackwidthX() / 2.0,
            drivetrainConfig.trackwidthY() / 2.0), // Rear right module
        new Translation2d(
            -drivetrainConfig.trackwidthX() / 2.0,
            -drivetrainConfig.trackwidthY() / 2.0) // Rear left module
      };
  // Define the kinematics for the swerve drive
  public static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(moduleTranslations);
  // Define the odometry frequency based on the robot type
  public static final double odometryFrequency =
      switch (Constants.getRobot()) {
        case SIMBOT -> 50.0; // For simulation robot
        case COMPBOT -> 50.0; // For competition robot
      };
  // Define the state standard deviations based on the robot type
  public static final Matrix<N3, N1> stateStdDevs =
      switch (Constants.getRobot()) {
        default ->
            new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.0002)); // Default standard deviations
      };
  // Define the xy standard deviation coefficient based on the robot type
  public static final double xyStdDevCoefficient =
      switch (Constants.getRobot()) {
        default -> 0.01; // Default coefficient
      };
  // Define the theta standard deviation coefficient based on the robot type
  public static final double thetaStdDevCoefficient =
      switch (Constants.getRobot()) {
        default -> 0.01; // Default coefficient
      };
  // Define the gyro ID
  public static final int gyroID = 30;
  // Define the canbus name
  public static final String canbus = "rio"; // Turn to "" for no canbus name
  // Define the module configurations based on the robot type
  public static ModuleConfig[] moduleConfigs =
      switch (Constants.getRobot()) {
        case COMPBOT -> // For competition robot
            new ModuleConfig[] {
              new ModuleConfig(
                  1,
                  2,
                  9,
<<<<<<< HEAD
                  Rotation2d.fromRotations(0.022949).plus(Rotation2d.fromDegrees(180)),
                  true), // Front right module
              new ModuleConfig(
                  3, 4, 10, Rotation2d.fromRotations(-0.412842), true), // Front left module
=======
                  Rotation2d.fromRotations(0.028809 - 0.01).plus(Rotation2d.fromDegrees(180)),
                  true),
              new ModuleConfig(
                  3,
                  4,
                  10,
                  Rotation2d.fromRotations(0.088623 - 0.01).plus(Rotation2d.fromDegrees(180)),
                  true),
>>>>>>> b05b67a (Auton Path and working arm distance (Arm positions need tuned))
              new ModuleConfig(
                  5,
                  6,
                  11,
<<<<<<< HEAD
                  Rotation2d.fromRotations(-0.245117).plus(Rotation2d.fromDegrees(180)),
                  true), // Rear right module
              new ModuleConfig(
                  7, 8, 12, Rotation2d.fromRotations(0.260742), true) // Rear left module
=======
                  Rotation2d.fromRotations(-0.246582).plus(Rotation2d.fromDegrees(180)),
                  true),
              new ModuleConfig(7, 8, 12, Rotation2d.fromRotations(0.255859), true)
>>>>>>> b05b67a (Auton Path and working arm distance (Arm positions need tuned))
            };
        case SIMBOT -> { // For simulation robot
          ModuleConfig[] configs = new ModuleConfig[4];
          for (int i = 0; i < configs.length; i++)
            configs[i] =
                new ModuleConfig(0, 0, 0, new Rotation2d(0), false); // Default module configuration
          yield configs;
        }
      };

  // Define the module constants based on the robot type
  public static final ModuleConstants moduleConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> // For competition robot
            new ModuleConstants(
                2.0, // Feedforward coefficient for the static friction
                0.0, // Feedforward coefficient for the velocity
                200.0, // Proportional gain for the drive
                0.0, // Derivative gain for the drive
                200.0, // Proportional gain for the turn
                20.0, // Derivative gain for the turn
                Mk4iReductions.L1.reduction, // Drive reduction
                Mk4iReductions.TURN.reduction); // Turn reduction
        case SIMBOT -> // For simulation robot
            new ModuleConstants(
                0.014, // Feedforward coefficient for the static friction
                0.134, // Feedforward coefficient for the velocity
                0.1, // Proportional gain for the drive
                0.0, // Derivative gain for the drive
                10.0, // Proportional gain for the turn
                0.0, // Derivative gain for the turn
                Mk4iReductions.L1.reduction, // Drive reduction
                Mk4iReductions.TURN.reduction); // Turn reduction
      };

  // Define the heading controller constants based on the robot type
  public static HeadingControllerConstants headingControllerConstants =
      switch (Constants.getRobot()) {
        case COMPBOT -> new HeadingControllerConstants(3.0, 0.0); // For competition robot
        case SIMBOT -> new HeadingControllerConstants(3.0, 0.0); // For simulation robot
      };

  // Define the PID constants for the path planner translation based on the robot type
  public static final PIDConstants PPtranslationConstants =
      switch (Constants.getRobot()) {
<<<<<<< HEAD
        case COMPBOT -> new PIDConstants(10, 0.0, 0.0); // For competition robot
        case SIMBOT -> new PIDConstants(10, 0.0, 0.0); // For simulation robot
=======
        case COMPBOT -> new PIDConstants(5, 0.0, 0.0);
        case SIMBOT -> new PIDConstants(10, 0.0, 0.0);
>>>>>>> b05b67a (Auton Path and working arm distance (Arm positions need tuned))
      };

  // Define the PID constants for the path planner rotation based on the robot type
  public static final PIDConstants PProtationConstants =
      switch (Constants.getRobot()) {
<<<<<<< HEAD
        case COMPBOT -> new PIDConstants(10, 0.0, 0.0); // For competition robot
        case SIMBOT -> new PIDConstants(10, 0.0, 0.0); // For simulation robot
=======
        case COMPBOT -> new PIDConstants(5, 0.0, 0.0);
        case SIMBOT -> new PIDConstants(10, 0.0, 0.0);
>>>>>>> b05b67a (Auton Path and working arm distance (Arm positions need tuned))
      };

  // Define the drivetrain configuration
  public record DrivetrainConfig(
      double wheelRadius,
      double trackwidthX,
      double trackwidthY,
      double maxLinearVelocity,
      double maxLinearAcceleration,
      double maxAngularVelocity,
      double maxAngularAcceleration) {
    // Calculate the drive base radius
    public double driveBaseRadius() {
      return Math.hypot(trackwidthX / 2.0, trackwidthY / 2.0);
    }
  }

  // Define the module configuration
  public record ModuleConfig(
      int driveID,
      int turnID,
      int absoluteEncoderChannel,
      Rotation2d absoluteEncoderOffset,
      boolean turnMotorInverted) {}

  // Define the module constants
  public record ModuleConstants(
      double ffKs,
      double ffKv,
      double driveKp,
      double drivekD,
      double turnKp,
      double turnkD,
      double driveReduction,
      double turnReduction) {}

  // Define the heading controller constants
  public record HeadingControllerConstants(double Kp, double Kd) {}

  // Define the reductions for the Mk4i
  private enum Mk4iReductions {
    L1((50.0 / 14.0) * (19.0 / 25.0) * (45.0 / 15.0)), // Reduction for level 1
    L2((50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0)), // Reduction for level 2
    L3((50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0)), // Reduction for level 3
    TURN((150.0 / 7.0)); // Reduction for turn

    final double reduction;

    Mk4iReductions(double reduction) {
      this.reduction = reduction;
    }
  }
}
