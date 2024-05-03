// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.lineBreak;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.lineBreak.LineBreakIO.LineBreakIOInputs;
import frc.robot.util.visualizer.RobotGamePieceVisualizer;
import org.littletonrobotics.junction.AutoLogOutput;

public class LineBreak extends SubsystemBase {
  private final LineBreakIO lineBreakIO;
  private final LineBreakIOInputs inputs = new LineBreakIOInputs();
  private double lastGamePieceSeenTime;

  /** Creates a new LineBreak. */
  public LineBreak(LineBreakIO lineBreakIO) {
    this.lineBreakIO = lineBreakIO;
    lastGamePieceSeenTime = Timer.getFPGATimestamp();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    lineBreakIO.updateInputs(inputs);
    if (inputs.lineBreakValues.hasGamePiece()) {
      lastGamePieceSeenTime = Timer.getFPGATimestamp();
    }
  }

  @AutoLogOutput(key = "/LineBreak/timeSinceLastGamePiece")
  public double timeSinceLastGamePiece() {
    return Timer.getFPGATimestamp() - lastGamePieceSeenTime;
  }

  @AutoLogOutput(key = "/LineBreak/hasNoGamePiece")
  public boolean hasNoGamePiece() {
    return !hasGamePiece();
  }

  @AutoLogOutput(key = "/LineBreak/hasGamePiece")
  public boolean hasGamePiece() {
    return inputs.lineBreakValues.hasGamePiece();
  }

  @AutoLogOutput(key = "/LineBreak/hasGamePieceIntake")
  public boolean hasGamePieceIntake() {
    return inputs.lineBreakValues.hasGamePieceIntake();
  }

  @AutoLogOutput(key = "/LineBreak/notInIntake")
  public boolean notInIntake() {
    return inputs.lineBreakValues.notInIntake();
  }

  @AutoLogOutput(key = "/LineBreak/inIntake")
  public boolean inIntake() {
    return inputs.lineBreakValues.inIntake();
  }

  @AutoLogOutput(key = "/LineBreak/isShooterLoaded")
  public boolean isShooterLoaded() {
    return inputs.lineBreakValues.isShooterLoaded();
  }

  public void bumpGamePiece() {
    lineBreakIO.bumpGamePiece();
    RobotGamePieceVisualizer.drawGamePieces();
  }

  public void shootGamePiece() {
    lineBreakIO.shootGamePiece();
  }

  public void setGamePiece(boolean intake) {
    lineBreakIO.setGamePiece(intake);
    RobotGamePieceVisualizer.drawGamePieces();
  }
}
