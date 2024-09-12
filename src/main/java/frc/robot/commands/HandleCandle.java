// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.beambreak.BeamBreak;
import frc.robot.subsystems.leds.Candle;

public class HandleCandle extends Command {
  Candle candle;
  BeamBreak beamBreak;

  /** Creates a new HandleLEDs. */
  public HandleCandle(Candle candle, BeamBreak beamBreak) {
    this.candle = candle;
    this.beamBreak = beamBreak;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(candle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (beamBreak.hasGamePiece()) {
      candle.setHasGamePiece(true);
    } else {
      candle.setHasGamePiece(false);
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