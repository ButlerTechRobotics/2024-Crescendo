// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.SmartController;
import frc.robot.SmartController.DriveModeType;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.beambreak.BeamBreak;
import frc.robot.subsystems.magazine.Magazine;
import frc.robot.subsystems.shooter.Shooter;

public class ManualShoot extends Command {
  Arm arm;
  Shooter shooter;
  Magazine magazine;
  BeamBreak beamBreak;
  Timer timer;
  Timer shooterTimer;
  double forceShootTimeout;

  /** Creates a new Shoot. */
  public ManualShoot(
      Shooter shooter, Magazine magazine, BeamBreak beamBreak, double forceShootTimeout) {
    this.shooter = shooter;
    this.magazine = magazine;
    this.beamBreak = beamBreak;
    timer = new Timer();
    shooterTimer = new Timer();
    this.forceShootTimeout = forceShootTimeout;
    addRequirements(magazine, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartController.getInstance().enableSmartControl();
    // if (Constants.getMode() == Constants.Mode.SIM) timer.restart();
    // flywheel.setSpeedRotPerSec(30);
    if (SmartController.getInstance().getDriveModeType() == DriveModeType.DEMO) {
      shooter.setSetpoint(2500, 2500);
    }
    shooterTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooterTimer.hasElapsed(0.5)) {
      magazine.shoot();
      if (Constants.getMode() == Constants.Mode.SIM && timer.hasElapsed(0.75)) {
        beamBreak.shootGamePiece();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartController.getInstance().disableSmartControl();
    magazine.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterTimer.hasElapsed(1.5);
  }
}
