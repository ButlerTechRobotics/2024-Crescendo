// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.leds.LedController;
import frc.robot.subsystems.linebreak.LineBreak;

public class HandleLEDs extends Command {
  LedController ledController;
  LineBreak lineBreak;

  /** Creates a new HandleLEDs. */
  public HandleLEDs(LedController ledController, LineBreak lineBreak) {
    this.ledController = ledController;
    this.lineBreak = lineBreak;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ledController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (lineBreak.hasGamePiece()) {
      ledController.setHasGamePiece(true);
    } else {
      ledController.setHasGamePiece(false);
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
