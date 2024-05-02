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
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.linebreak.LineBreak;
import frc.robot.subsystems.magazine.Magazine;

public class SmartMagazine extends Command {
  Magazine magazine;
  LineBreak lineBreak;
  Intake intake;

  /** Creates a new SmartMagazine. */
  public SmartMagazine(Magazine magazine, Intake intake, LineBreak lineBreak) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.magazine = magazine;
    this.lineBreak = lineBreak;
    this.intake = intake;
    addRequirements(magazine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartController.getInstance().getDriveModeType() == DriveModeType.SAFE
        || lineBreak.isShooterLoaded()
        || (lineBreak.hasNoGamePiece() && !intake.getIntakeRequest())) {
      magazine.stop();
      return;
    }

    if (lineBreak.inIntake()) {
      magazine.slowBackward();
    } else {
      if (intake.getIntakeRequest()) {
        magazine.forward();

      } else {
        if (lineBreak.notInIntake()) {
          magazine.slowForward();
        }
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
