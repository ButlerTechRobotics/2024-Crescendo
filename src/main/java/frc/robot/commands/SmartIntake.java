// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.beambreak.BeamBreak;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Candle;
import frc.robot.subsystems.magazine.Magazine;

public class SmartIntake extends Command {

  Intake intake;
  Magazine magazine;
  BeamBreak beamBreak;
  Candle candle;

  /** Creates a new IntakeDown. */
  public SmartIntake(Intake intake, BeamBreak beamBreak, Magazine magazine, Candle candle) {
    this.intake = intake;
    this.beamBreak = beamBreak;
    this.magazine = magazine;
    this.candle = candle;

    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((intake.getIntakeRequest() && beamBreak.hasNoGamePiece()) || intake.getOuttakeRequest()) {
      if (intake.getIntakeRequest() && beamBreak.hasNoGamePiece()) {
        intake.intake();
        candle.prettyLights();
      } else if (intake.getOuttakeRequest()) {
        intake.outtake();
      }
    } else {
      if (!beamBreak.isShooterLoaded() && beamBreak.hasGamePiece()) {
        intake.stop();
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
