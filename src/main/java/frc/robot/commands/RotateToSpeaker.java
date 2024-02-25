package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.util.FieldConstants;

public class RotateToSpeaker extends CommandBase {
  private final Drive drivetrain;
  private final AprilTagVision aprilTagVision;
  private final Pose2d speakerPosition;

  public RotateToSpeaker(Drive drivetrain, AprilTagVision aprilTagVision) {
    this.drivetrain = drivetrain;
    this.aprilTagVision = aprilTagVision;
    this.speakerPosition = FieldConstants.Speaker.centerSpeakerOpening;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    // Get the current position of the drivetrain
    Pose2d drivetrainPosition = drivetrain.getPose();

    // Calculate the angle between the drivetrain and the speaker using the calculateAngleToTarget
    // method
    double angle = aprilTagVision.calculateAngleToTarget(drivetrainPosition, speakerPosition);

    // Rotate the drivetrain to that angle
    drivetrain.rotateTo(angle);
  }

  @Override
  public boolean isFinished() {
    return true; // This command finishes immediately after rotating the drivetrain
  }
}
