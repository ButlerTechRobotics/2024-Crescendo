// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.flywheel.Flywheel;
import frc.robot.subsystems.lineBreak.LineBreak;

public class AutoPreRoll extends Command {
  Arm arm;
  Flywheel flywheel;
  LineBreak lineBreak;
  Rotation2d wristPosition;
  double flyWheelSpeed;

  /** Creates a new Shoot. */
  public AutoPreRoll(
      Arm arm,
      Flywheel flywheel,
      LineBreak lineBreak,
      Rotation2d wristPosition,
      double flyWheelSpeed) {
    this.arm = arm;
    this.flywheel = flywheel;
    this.lineBreak = lineBreak;
    this.wristPosition = wristPosition;
    this.flyWheelSpeed = flyWheelSpeed;
    addRequirements(arm, flywheel);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lineBreak.hasGamePiece()) {
      arm.setArmTarget(wristPosition.getRadians());
      if (flyWheelSpeed == 0.0) {
        flywheel.stop();
      } else {
        flywheel.setSetpoint(flyWheelSpeed, flyWheelSpeed);
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
