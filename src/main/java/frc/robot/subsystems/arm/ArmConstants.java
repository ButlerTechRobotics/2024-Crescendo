package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;

public final class ArmConstants {
  public static final ArmPositions amp = new ArmPositions(Rotation2d.fromDegrees(82));
  public static final ArmPositions intake = new ArmPositions(Rotation2d.fromDegrees(0));
  public static final ArmPositions shoot = new ArmPositions(Rotation2d.fromDegrees(0));
  public static final ArmPositions climb = new ArmPositions(Rotation2d.fromDegrees(0));
  public static final ArmPositions feed = new ArmPositions(Rotation2d.fromDegrees(0));
  public static final ArmPositions manualShot = new ArmPositions(Rotation2d.fromDegrees(40));

  public static final MotorFeedbackController armControlConstants =
      switch (Constants.getRobot()) {
        case SIMBOT -> new MotorFeedbackController(0, 0, 0, 0);
        case COMPBOT -> new MotorFeedbackController(0.055, 0.0001, 0.0, 0.0);
          // case COMPBOT -> new MotorFeedbackController(0, 0, 0, 0);
      };

  public static final int ARM_GEAR_RATIO = 125;

  public record ArmPositions(Rotation2d arm) {}

  public record MotorFeedbackController(double kP, double kI, double kD, double kG) {}
}
