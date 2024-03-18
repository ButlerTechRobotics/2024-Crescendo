// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveController;
import java.util.function.DoubleSupplier;

public class DriveCommands {
  private static final double DEADBAND = 0.1;

  private DriveCommands() {
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and
   * angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DriveController driveMode,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return Commands.run(
        () -> {
          // Apply deadband
          double linearMagnitude = MathUtil.applyDeadband(
              Math.hypot(xSupplier.getAsDouble(), ySupplier.getAsDouble()), DEADBAND);
          Rotation2d linearDirection = new Rotation2d(xSupplier.getAsDouble(), ySupplier.getAsDouble());
          double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

          // // If the joystick is not moving, stop the drive and turn the modules to an X
          // // arrangement
          // if (Math.abs(linearMagnitude) < DEADBAND && Math.abs(omega) < DEADBAND) {
          //   drive.stopWithX();
          //   return;
          // }

          if (driveMode.isHeadingControlled()) {
            final var targetAngle = driveMode.getHeadingAngle();
            omega = Drive.getThetaController()
                .calculate(
                    drive.getPose().getRotation().getRadians(), targetAngle.get().getRadians());
            if (Drive.getThetaController().atGoal()) {
              omega = 0;
            }
            omega = Math.copySign(Math.min(1, Math.abs(omega)), omega);
          }

          // Square values
          linearMagnitude = linearMagnitude * linearMagnitude;
          omega = Math.copySign(omega * omega, omega);

          // Calcaulate new linear velocity
          Translation2d linearVelocity = new Pose2d(new Translation2d(), linearDirection)
              .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
              .getTranslation();

          // Convert to field relative speeds & send command
          boolean isFlipped = DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
          drive.runVelocity(
              ChassisSpeeds.fromFieldRelativeSpeeds(
                  linearVelocity.getX() * drivetrainConfig.maxLinearVelocity(),
                  linearVelocity.getY() * drivetrainConfig.maxLinearVelocity(),
                  omega * drivetrainConfig.maxAngularVelocity(),
                  isFlipped
                      ? drive.getRotation().plus(new Rotation2d(Math.PI))
                      : drive.getRotation()));
        },
        drive);
  }
}
