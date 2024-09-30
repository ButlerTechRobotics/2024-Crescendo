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
import frc.robot.subsystems.magazine.Magazine;
import org.littletonrobotics.junction.Logger;

public class FeedShooter extends Command {
  Magazine magazine;
  BeamBreak beamBreak;
  Timer timer;
  Timer shooterTimer;
  double shootDelay;

  public FeedShooter(Magazine magazine, BeamBreak beambreak, double shootDelay) {
    this.magazine = magazine;
    this.beamBreak = beambreak;
    this.shootDelay = shootDelay;
    addRequirements(magazine, beambreak);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    boolean isShooting = false;
    double realShootDelay = shootDelay;

    if (shooterTimer.hasElapsed(realShootDelay)) {
      magazine.shoot();
      isShooting = true;
    }

    Logger.recordOutput("FeedShooter/IsShooting", isShooting);
  }
}
