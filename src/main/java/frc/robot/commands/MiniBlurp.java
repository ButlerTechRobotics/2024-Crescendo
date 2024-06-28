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
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.beambreak.BeamBreak;
import frc.robot.subsystems.magazine.Magazine;
import frc.robot.subsystems.shooter.Shooter;

public class MiniBlurp extends Command {
  Arm arm;
  Shooter shooter;
  Magazine magazine;
  BeamBreak beamBreak;
  Timer timer;
  Timer flywheelTimer;
  double forceShootTimeout;

  /** Creates a new Shoot. */
  public MiniBlurp(
      Arm arm, Shooter shooter, Magazine magazine, BeamBreak beamBreak, double forceShootTimeout) {
    this.arm = arm;
    this.shooter = shooter;
    this.magazine = magazine;
    this.beamBreak = beamBreak;
    timer = new Timer();
    flywheelTimer = new Timer();
    this.forceShootTimeout = forceShootTimeout;
    addRequirements(magazine, arm, shooter);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartController.getInstance().enableSmartControl();
    // if (Constants.getMode() == Constants.Mode.SIM) timer.restart();
    // flywheel.setSpeedRotPerSec(30);
    shooter.setSetpoint(1500, 1500);
    arm.setArmTargetAngle(ArmConstants.home.arm().getDegrees());
    flywheelTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((arm.atSetpoint() && (shooter.atTargetSpeed() || flywheelTimer.hasElapsed(0.5)))
        || flywheelTimer.hasElapsed(forceShootTimeout)) {
      magazine.shoot();
      if (Constants.getMode() == Constants.Mode.SIM && timer.hasElapsed(0.5)) {
        beamBreak.shootGamePiece();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    magazine.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return beamBreak.hasNoGamePiece() && beamBreak.timeSinceLastGamePiece() > 1.0;
  }
}
