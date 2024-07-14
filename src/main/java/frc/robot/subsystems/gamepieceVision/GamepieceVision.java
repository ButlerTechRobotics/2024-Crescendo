// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.gamepieceVision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class GamepieceVision extends SubsystemBase {

  private final GamepieceVisionIO io;
  private final GamepieceVisionIOInputsAutoLogged inputs;

  public GamepieceVision(GamepieceVisionIO io) {
    System.out.println("[Init] Creating GamepieceVision");
    this.io = io;
    this.inputs = new GamepieceVisionIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    long startTime = Logger.getRealTimestamp();

    if (!Logger.hasReplaySource()) {
      io.updateInputs(inputs);
    }

    Logger.processInputs("GamepieceVision", inputs);

    if (hasTarget()) {
      Logger.recordOutput(
          "GamepieceVision/EstimatedFieldPos",
          new Pose3d(getEstimatedFieldPos(), new Rotation3d()));
    } else {
      Logger.recordOutput("GamepieceVision/EstimatedFieldPos", new Pose3d());
    }

    double runtimeMS = (Logger.getRealTimestamp() - startTime) / 1000.0;
    Logger.recordOutput("GamepieceVision/PeriodicRuntimeMS", runtimeMS);
  }

  public boolean hasTarget() {
    return inputs.hasTarget;
  }

  public Rotation2d getAngleToTarget() {
    return Rotation2d.fromDegrees(-inputs.tx);
  }

  public Rotation2d getTY() {
    return Rotation2d.fromDegrees(inputs.ty);
  }

  public double getMeasurementTimestamp() {
    return inputs.timestamp;
  }

  public Translation3d getEstimatedFieldPos() {
    Transform3d cameraPose = GamepieceVisionCameraConstants.ROBOT_TO_CAMERA_NOTES;

    Logger.recordOutput("GamepieceVision/CameraPose", cameraPose);

    double estDistance =
        Math.abs(
            (cameraPose.getZ() - 0.025)
                / Math.sin(Units.degreesToRadians(inputs.ty) - cameraPose.getRotation().getY()));
    Translation3d llRelative =
        new Translation3d(
            estDistance,
            new Rotation3d(
                0.0, Units.degreesToRadians(-inputs.ty), Units.degreesToRadians(-inputs.tx)));

    Translation3d fieldRelative =
        llRelative.rotateBy(cameraPose.getRotation()).plus(cameraPose.getTranslation());
    return new Translation3d(fieldRelative.getX(), fieldRelative.getY(), 0.025);
  }
}
