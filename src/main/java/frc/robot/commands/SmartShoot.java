// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.Rollers.Goal;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class SmartShoot extends Command {
  Arm arm;
  Shooter shooter;
  Rollers rollers;
  Supplier<Pose2d> pose;
  Timer timer;
  Timer shooterTimer;
  double forceShootTimeout;

  /** Creates a new Shoot. */
  public SmartShoot(
      Arm arm, Shooter shooter, Rollers rollers, Supplier<Pose2d> pose, double forceShootTimeout) {
    this.arm = arm;
    this.shooter = shooter;
    this.rollers = rollers;
    this.pose = pose;
    timer = new Timer();
    shooterTimer = new Timer();
    this.forceShootTimeout = forceShootTimeout;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartController.getInstance().enableSmartControl();
    if (Constants.getMode() == Constants.Mode.SIM) timer.restart();
    shooterTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean isSmartControlEnabled = SmartController.getInstance().isSmartControlEnabled();
    boolean isWristInTargetPose = isArmInTargetPose();
    boolean isDriveAngleInTarget = isDriveAngleInTarget();
    boolean isShooterAtTargetSpeed = isShooterAtTargetSpeed();
    boolean isShooting = false;
    double realForceShoot = forceShootTimeout;

    if ((isSmartControlEnabled
            && isWristInTargetPose
            && isDriveAngleInTarget
            && isShooterAtTargetSpeed)
        || shooterTimer.hasElapsed(realForceShoot)) {
      rollers.setGoal(Goal.SHOOT);
      isShooting = true;
    }
    Logger.recordOutput("SmartShoot/IsShooting", isShooting);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartController.getInstance().disableSmartControl();
    rollers.setGoal(Goal.IDLE);
    SmartController.getInstance().setDriveMode(DriveModeType.SAFE);
  }

  // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  // if (DriverStation.isAutonomous()) {
  // return lineBreak.hasNoGamePiece() && lineBreak.timeSinceLastGamePiece() >
  // 0.1;
  // }
  // return lineBreak.hasNoGamePiece() && lineBreak.timeSinceLastGamePiece() >
  // 0.5;
  // }

  public boolean isArmInTargetPose() {
    Logger.recordOutput("SmartShoot/IsArmAtSetpoint", arm.atSetpoint());
    Logger.recordOutput("SmartShoot/ArmAngle", arm.getArmCurrentAngle());
    Logger.recordOutput("SmartShoot/TargetArmAngle", arm.getArmTargetAngle());
    return arm.atSetpoint();
  }

  public boolean isShooterAtTargetSpeed() {
    Logger.recordOutput("SmartShoot/IsShooterAtTargetSpeed", shooter.atTargetSpeed());
    Logger.recordOutput("SmartShoot/TopShooterSpeed", shooter.getTopCharacterizationVelocity());
    Logger.recordOutput(
        "SmartShoot/BottomShooterSpeed", shooter.getBottomCharacterizationVelocity());
    Logger.recordOutput(
        "SmartShoot/TargetShooterSpeed",
        SmartController.getInstance().getTargetAimingParameters().shooterSpeed());
    return shooter.atTargetSpeed();
  }

  public boolean isDriveAngleInTarget() {
    Rotation2d targetAngle = SmartController.getInstance().getTargetAimingParameters().robotAngle();
    Rotation2d robotAngle = this.pose.get().getRotation();
    boolean isDriveAngleInTarget =
        Math.abs(robotAngle.minus(targetAngle).getRadians()) < Units.degreesToRadians(1.5);
    Logger.recordOutput("SmartShoot/IsDriveAngleInTarget", isDriveAngleInTarget);
    Logger.recordOutput("SmartShoot/DriveAngle", robotAngle.getDegrees());
    Logger.recordOutput("SmartShoot/TargetDriveAngle", targetAngle.getDegrees());
    return isDriveAngleInTarget;
  }
}