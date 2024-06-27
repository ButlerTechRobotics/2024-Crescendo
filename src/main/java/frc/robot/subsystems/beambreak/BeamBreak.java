// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.beambreak;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.beambreak.BeamBreakIO.BeamBreakIOInputs;
// import frc.robot.util.visualizer.RobotGamePieceVisualizer;
import org.littletonrobotics.junction.AutoLogOutput;

public class BeamBreak extends SubsystemBase {
  private final BeamBreakIO beamBreakIO;
  private final BeamBreakIOInputs inputs = new BeamBreakIOInputs();
  private double lastGamePieceSeenTime;

  /** Creates a new BeamBreak. */
  public BeamBreak(BeamBreakIO beamBreakIO) {
    this.beamBreakIO = beamBreakIO;
    lastGamePieceSeenTime = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    beamBreakIO.updateInputs(inputs);
    if (inputs.beamBreakValues.hasGamePiece()) {
      lastGamePieceSeenTime = Timer.getFPGATimestamp();
    }
  }

  @AutoLogOutput(key = "/BeamBreak/timeSinceLastGamePiece")
  public double timeSinceLastGamePiece() {
    return Timer.getFPGATimestamp() - lastGamePieceSeenTime;
  }

  @AutoLogOutput(key = "/BeamBreak/hasNoGamePiece")
  public boolean hasNoGamePiece() {
    return !hasGamePiece();
  }

  @AutoLogOutput(key = "/BeamBreak/hasGamePiece")
  public boolean hasGamePiece() {
    return inputs.beamBreakValues.hasGamePiece();
  }

  // TODO once we add a laser for the intake, change this to the intake's laser
  @AutoLogOutput(key = "/BeamBreak/HasNoteForAuto")
  public boolean hasNoteForAuto() {
    return inputs.beamBreakValues.hasGamePiece();
  }

  @AutoLogOutput(key = "/BeamBreak/isShooterLoaded")
  public boolean isShooterLoaded() {
    return inputs.beamBreakValues.isShooterLoaded();
  }

  public void shootGamePiece() {
    beamBreakIO.shootGamePiece();
  }

  public void setGamePiece(boolean magazine) {
    beamBreakIO.setGamePiece(magazine);
    // RobotGamePieceVisualizer.drawGamePieces();
  }
}
