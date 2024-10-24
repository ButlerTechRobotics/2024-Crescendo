// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
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
  Timer flywheelTimer;
  double forceShootTimeout;
  Rotation2d armAngle;
  double flywheelSpeed;

  /** Creates a new Shoot. */
  public ManualShoot(
      Arm arm,
      Shooter shooter,
      Magazine magazine,
      BeamBreak beamBreak,
      Rotation2d armAngle,
      double flywheelSpeed,
      double forceShootTimeout) {
    this.arm = arm;
    this.shooter = shooter;
    this.magazine = magazine;
    this.beamBreak = beamBreak;
    this.armAngle = armAngle;
    this.flywheelSpeed = flywheelSpeed;
    this.forceShootTimeout = forceShootTimeout;
    timer = new Timer();
    flywheelTimer = new Timer();
    addRequirements(arm, shooter, magazine);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.stop();
    shooter.stop();
    magazine.stop();
    flywheelTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.setArmTargetAngle(armAngle.getDegrees());
    shooter.setSetpoint(flywheelSpeed, flywheelSpeed);
    // Check if the shooter is ready to shoot
    if (flywheelTimer.hasElapsed(0.25) || flywheelTimer.hasElapsed(forceShootTimeout)) {
      // Run the magazine to shoot the game piece
      magazine.shoot();

      // Simulate shooting in simulation mode
      if (Constants.getMode() == Constants.Mode.SIM && timer.hasElapsed(0.25)) {
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
    // End the command once the game piece has been shot and is no longer detected
    return beamBreak.hasNoGamePiece() && beamBreak.timeSinceLastGamePiece() > 1.0;
  }
}
