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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.linebreak.LineBreak;
import frc.robot.subsystems.magazine.Magazine;
import java.util.function.BooleanSupplier;

public class SmartIntake extends Command {

  Intake intake;
  Arm arm;
  Magazine magazine;
  Boolean runArmDown;
  BooleanSupplier isArmWristInIntakePosition;
  LineBreak lineBreak;

  /** Creates a new IntakeDown. */
  public SmartIntake(
      Intake intake,
      Arm arm,
      LineBreak lineBreak,
      Magazine magazine,
      BooleanSupplier isArmWristInIntakePosition) {
    this.intake = intake;
    this.arm = arm;
    this.isArmWristInIntakePosition = isArmWristInIntakePosition;
    this.lineBreak = lineBreak;
    this.magazine = magazine;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartController.getInstance().getDriveModeType() == DriveModeType.SAFE) {
      arm.setArmTarget(0);
      intake.stop();
      return;
    }
    if (magazine.isShooting()) {
      intake.intakeSlow();
      return;
    }
    if (intake.getIntakeRequest() && lineBreak.hasNoGamePiece()) {
      arm.setArmTarget(0);
      intake.intake();
    } else {
      arm.setArmTarget(0);
      if (!(lineBreak.isShooterLoaded() || lineBreak.inIntake()) && lineBreak.hasGamePiece()) {
        intake.intake();
      } else {
        intake.stop();
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
