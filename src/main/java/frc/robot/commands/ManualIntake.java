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
import frc.robot.subsystems.arm.ArmConstants.ArmPositions;
import frc.robot.subsystems.linebreak.LineBreak;
import frc.robot.subsystems.shooter.Shooter;
import java.util.function.DoubleSupplier;

public class ManualIntake extends Command {
  Arm arm;
  Shooter shooter;
  ArmPositions armPosition;
  DoubleSupplier flywheelSpeed;
  LineBreak lineBreak;

  /** Creates a new moveArm. */
  public ManualIntake(
      Arm arm,
      Shooter shooter,
      ArmPositions armPosition,
      DoubleSupplier flywheelSpeed,
      LineBreak lineBreak) {
    this.arm = arm;
    this.shooter = shooter;
    this.armPosition = armPosition;
    this.flywheelSpeed = flywheelSpeed;
    this.lineBreak = lineBreak;
    addRequirements(arm, shooter);
  }

  @Override
  public void initialize() {
    arm.setArmTarget(armPosition.arm().getDegrees());
    if (flywheelSpeed.getAsDouble() == 0.0) {
      shooter.stop();
    } else {
      shooter.setSetpoint(flywheelSpeed.getAsDouble(), flywheelSpeed.getAsDouble());
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stop();
    arm.setArmTarget(ArmConstants.intake.arm().getDegrees());
    SmartController.getInstance().setDriveMode(DriveModeType.SPEAKER);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return lineBreak.isShooterLoaded();
  }
}
