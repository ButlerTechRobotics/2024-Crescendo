// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.linebreak.LineBreak;

public class SmartArm extends Command {
  Arm arm;
  LineBreak lineBreak;

  /** Creates a new moveArm. */
  public SmartArm(Arm arm, LineBreak lineBreak) {
    this.arm = arm;
    this.lineBreak = lineBreak;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    DriveModeType driveModeType = SmartController.getInstance().getDriveModeType();
    boolean isSmartControlled = SmartController.getInstance().isSmartControlEnabled();
    if (driveModeType == DriveModeType.SAFE) {
      arm.stop();
      return;
    }
    if ((lineBreak.isShooterLoaded()) && isSmartControlled) {
      if (driveModeType == DriveModeType.AMP) {
        arm.setArmTarget(ArmConstants.amp.arm().getDegrees());
        return;
      }
      if (driveModeType == DriveModeType.SPEAKER || driveModeType == DriveModeType.FEED) {
        arm.setArmTarget(ArmConstants.shoot.arm().getDegrees());
        return;
      }
    } else if ((SmartController.getInstance().getDriveModeType() == DriveModeType.SPEAKER
            || SmartController.getInstance().getDriveModeType() == DriveModeType.FEED)
        && SmartController.getInstance().getTargetAimingParameters().effectiveDistanceToTarget()
            < SmartController.getInstance().getPrerollDistance()
        && (lineBreak.isShooterLoaded())) {
      arm.setArmTarget(ArmConstants.shoot.arm().getDegrees());
      return;
    }
    if (lineBreak.timeSinceLastGamePiece() > 0.5
        || ((lineBreak.isShooterLoaded()) && !isSmartControlled)) {
      arm.setArmTarget(ArmConstants.intake.arm().getDegrees());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
