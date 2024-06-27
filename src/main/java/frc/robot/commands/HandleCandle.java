// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.beambreak.BeamBreak;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.leds.Candle;

public class HandleCandle extends Command {
  Candle candle;
  BeamBreak beamBreak;
  Intake intake;

  /** Creates a new HandleLEDs. */
  public HandleCandle(Candle candle, BeamBreak beamBreak, Intake intake) {
    this.candle = candle;
    this.beamBreak = beamBreak;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(candle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (beamBreak.hasGamePiece()) {
      candle.setHasGamePiece(true);
    } else {
      candle.setHasGamePiece(false);
    }

    if (intake.getIntakeRequest()) {
      candle.setIntakeRequest(true);
    } else {
      candle.setIntakeRequest(false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}