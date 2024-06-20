// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.beambreak.BeamBreak;
import frc.robot.subsystems.magazine.Magazine;

public class ManualMagazine extends Command {
  Magazine magazine;
  BeamBreak beamBreak;
  Timer timer;

  /** Creates a new SmartMagazine. */
  public ManualMagazine(Magazine magazine, BeamBreak beamBreak) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.magazine = magazine;
    this.beamBreak = beamBreak;
    this.timer = new Timer();
    addRequirements(magazine);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartController.getInstance().getDriveModeType() == DriveModeType.SAFE
        || beamBreak.isShooterLoaded()
        || beamBreak.hasNoGamePiece()) {
      magazine.stop();
      return;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return beamBreak.isShooterLoaded() || (timer.hasElapsed(3) && beamBreak.hasNoGamePiece());
  }
}
