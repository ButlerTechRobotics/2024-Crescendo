// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.linebreak.LineBreak;
import frc.robot.subsystems.shooter.Shooter;

public class AutoPreRoll extends Command {
  Arm arm;
  Shooter shooter;
  LineBreak lineBreak;
  Rotation2d armPosition;
  double flyWheelSpeed;

  /** Creates a new Shoot. */
  public AutoPreRoll(
      Arm arm, Shooter shooter, LineBreak lineBreak, Rotation2d armPosition, double flyWheelSpeed) {
    this.arm = arm;
    this.shooter = shooter;
    this.lineBreak = lineBreak;
    this.armPosition = armPosition;
    this.flyWheelSpeed = flyWheelSpeed;
    addRequirements(arm, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lineBreak.hasGamePiece()) {
      arm.setArmTarget(armPosition.getDegrees());
      if (flyWheelSpeed == 0.0) {
        shooter.stop();
      } else {
        shooter.setSetpoint(flyWheelSpeed, flyWheelSpeed);
      }
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
