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
import frc.robot.subsystems.shooter.Shooter;

public class SmartShooter extends Command {
  Shooter shooter;

  /** Creates a new SmartShooter. */
  public SmartShooter(Shooter shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    /* Not needed */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartController.getInstance().getDriveModeType() == DriveModeType.SAFE) {
      shooter.stop();
      return;
    }
    if (SmartController.getInstance().isSmartControlEnabled()) {
      shooter.setSetpoint(
          SmartController.getInstance().getTargetAimingParameters().shooterSpeed(),
          SmartController.getInstance().getTargetAimingParameters().shooterSpeed());
      return;
    } else if ((SmartController.getInstance().getDriveModeType() == DriveModeType.SPEAKER
            || SmartController.getInstance().getDriveModeType() == DriveModeType.FEED)
        && SmartController.getInstance().getTargetAimingParameters().effectiveDistanceToTarget()
            < SmartController.getInstance().getPrerollDistance()) {
      shooter.setSetpoint(
          SmartController.getInstance().getTargetAimingParameters().shooterSpeed(),
          SmartController.getInstance().getTargetAimingParameters().shooterSpeed() - 300);
      return;
    } else {
      shooter.stop();
      return;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    /* No end to function */
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
