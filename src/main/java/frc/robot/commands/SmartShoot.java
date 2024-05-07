// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class SmartShoot extends Command {
  // // Supplier<Pose2d> pose;
  // // Arm arm;
  // // Shooter shooter;
  // // Feeder feeder;
  // // Timer timer;
  // // Timer shooterTimer;
  // // double forceShootTimeout;

  // // /** Creates a new Shoot. */
  // // public SmartShoot(
  // //     Arm arm,
  // //     Shooter shooter,
  // //     Feeder feeder,
  // //     Supplier<Pose2d> pose,
  // //     double forceShootTimeout) {
  // //   this.arm = arm;
  // //   this.shooter = shooter;
  // //   this.feeder = feeder;
  // //   this.pose = pose;
  // //   timer = new Timer();
  // //   shooterTimer = new Timer();
  // //   this.forceShootTimeout = forceShootTimeout;
  // // }

  // // // Called when the command is initially scheduled.
  // // @Override
  // // public void initialize() {
  // //   SmartController.getInstance().enableSmartControl();
  // //   if (Constants.getMode() == Constants.Mode.SIM) timer.restart();
  // //   shooterTimer.restart();
  // // }

  // // // Called every time the scheduler runs while the command is scheduled.
  // // @Override
  // // public void execute() {
  // //   boolean isSmartControlEnabled = SmartController.getInstance().isSmartControlEnabled();
  // //   boolean isWristInTargetPose = isWristInTargetPose();
  // //   boolean isDriveAngleInTarget = isDriveAngleInTarget();
  // //   boolean isShooterAtTargetSpeed = isShooterAtTargetSpeed();
  // //   boolean isShooting = false;
  // //   double realForceShoot = forceShootTimeout;
  // //   if (SmartController.getInstance().getDriveModeType() == DriveModeType.CLIMBER) {
  // //     realForceShoot = 0.0;
  // //   }
  // //   if ((isSmartControlEnabled
  // //           && isWristInTargetPose
  // //           && isDriveAngleInTarget
  // //           && isFlywheelAtTargetSpeed)
  // //       || flywheelTimer.hasElapsed(realForceShoot)) {
  // //     magazine.shoot();
  // //     isShooting = true;
  // //     if (Constants.getMode() == Constants.Mode.SIM && timer.hasElapsed(0.75)) {
  // //       lineBreak.shootGamePiece();
  // //     }
  // //   }
  // //   Logger.recordOutput("SmartShoot/IsShooting", isShooting);
  // // }

  // // // Called once the command ends or is interrupted.
  // // @Override
  // // public void end(boolean interrupted) {
  // //   SmartController.getInstance().disableSmartControl();
  // //   magazine.stop();
  // // }

  // // // Returns true when the command should end.
  // // @Override
  // // public boolean isFinished() {
  // //   if (DriverStation.isAutonomous()) {
  // //     return lineBreak.hasNoGamePiece() && lineBreak.timeSinceLastGamePiece() > 0.1;
  // //   }
  // //   return lineBreak.hasNoGamePiece() && lineBreak.timeSinceLastGamePiece() > 0.5;
  // // }

  // // public boolean isWristInTargetPose() {
  // //   Logger.recordOutput("SmartShoot/IsWristInTargetPose", arm.isArmWristInTargetPose());
  // //   Logger.recordOutput("SmartShoot/WristAngle", arm.getWristAngleRelative());
  // //   Logger.recordOutput("SmartShoot/TargetWristAngle", arm.getRelativeWristTarget());
  // //   return arm.isArmWristInTargetPose();
  // // }

  // // public boolean isFlywheelAtTargetSpeed() {
  // //   Logger.recordOutput("SmartShoot/IsFlywheelAtTargetSpeed", flywheel.atTargetSpeed());
  // //   Logger.recordOutput("SmartShoot/FlywheelSpeed", flywheel.getVelocityRotPerSec());
  // //   Logger.recordOutput(
  // //       "SmartShoot/TargetFlywheelSpeed",
  // //       SmartController.getInstance().getTargetAimingParameters().shooterSpeed());
  // //   return flywheel.atTargetSpeed();
  // // }

  // // public boolean isDriveAngleInTarget() {
  // //   Rotation2d targetAngle =
  // SmartController.getInstance().getTargetAimingParameters().robotAngle();
  // //   Rotation2d robotAngle = this.pose.get().getRotation();
  // //   boolean isDriveAngleInTarget =
  // //       Math.abs(robotAngle.minus(targetAngle).getRadians()) < Units.degreesToRadians(1.5);
  // //   Logger.recordOutput("SmartShoot/IsDriveAngleInTarget", isDriveAngleInTarget);
  // //   Logger.recordOutput("SmartShoot/DriveAngle", robotAngle.getDegrees());
  // //   Logger.recordOutput("SmartShoot/TargetDriveAngle", targetAngle.getDegrees());
  // //   return isDriveAngleInTarget;
  // }
}
