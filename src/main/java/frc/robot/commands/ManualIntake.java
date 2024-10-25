// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.beambreak.BeamBreak;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.magazine.Magazine;

public class ManualIntake extends Command {
  Intake intake;
  Magazine magazine;
  BeamBreak beamBreak;
  Timer timer;

  /** Creates a new SmartMagazine. */
  public ManualIntake(Intake intake, Magazine magazine, BeamBreak beamBreak) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.magazine = magazine;
    this.beamBreak = beamBreak;
    this.timer = new Timer();
    addRequirements(intake, magazine, beamBreak);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("ManualIntake command initialized.");
    timer.restart();
  }

  @Override
  public void execute() {
    if (beamBreak.hasGamePiece()) {
      System.out.println("Game piece detected. Stopping intake and magazine.");
      magazine.stop();
      intake.stop();
    } else {
      System.out.println("No game piece detected. Running intake and magazine.");
      magazine.intake();
      intake.intake();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ManualIntake command ended. Interrupted: " + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return beamBreak.isShooterLoaded() || (timer.hasElapsed(5) && beamBreak.hasNoGamePiece());
  }
}
