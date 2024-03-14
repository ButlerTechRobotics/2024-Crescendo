// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.intake.Intake;
import frc.robot.util.NoteVisualizer;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
  private final Intake intake;

  private final RollersSensorsIO sensorsIO;
  private final RollersSensorsIOInputsAutoLogged sensorInputs =
      new RollersSensorsIOInputsAutoLogged();

  public enum Goal {
    IDLE,
    FLOOR_INTAKE,
    EJECT_TO_FLOOR,
    FEED_TO_SHOOTER,
    AMP_SCORE,
    TRAP_SCORE,
    SHUFFLE_SHOOTER
  }

  public enum GamepieceState {
    NONE,
    SHOOTER_STAGED
  }

  @Getter private Goal goal = Goal.IDLE;
  @AutoLogOutput @Getter @Setter private GamepieceState gamepieceState = GamepieceState.NONE;
  private GamepieceState lastGamepieceState = GamepieceState.NONE;
  private Timer gamepieceStateTimer = new Timer();

  @Setter private BooleanSupplier backpackActuatedSupplier = () -> false;

  public Rollers(Intake intake, RollersSensorsIO sensorsIO) {
    this.intake = intake;
    this.sensorsIO = sensorsIO;

    setDefaultCommand(setGoalCommand(Goal.IDLE));
    gamepieceStateTimer.start();
  }

  @Override
  public void periodic() {
    sensorsIO.updateInputs(sensorInputs);
    Logger.processInputs("RollersSensors", sensorInputs);

    if (DriverStation.isDisabled()) {
      goal = Goal.IDLE;
    }

    if (sensorInputs.shooterStaged) {
      gamepieceState = GamepieceState.SHOOTER_STAGED;
    } else {
      gamepieceState = GamepieceState.NONE;
    }
    if (gamepieceState != lastGamepieceState) {
      gamepieceStateTimer.reset();
    }
    lastGamepieceState = gamepieceState;

    NoteVisualizer.setHasNote(gamepieceState != GamepieceState.NONE);

    // Reset idle and wait for other input
    intake.setGoal(Intake.Goal.IDLING);
    switch (goal) {
      case IDLE -> {}
      case FLOOR_INTAKE -> {
        if (gamepieceState == GamepieceState.SHOOTER_STAGED) {
          intake.setGoal(Intake.Goal.EJECTING);
        } else {
          intake.setGoal(Intake.Goal.FLOOR_INTAKING);
        }
      }
      case EJECT_TO_FLOOR -> {
        intake.setGoal(Intake.Goal.EJECTING);
      }
      case FEED_TO_SHOOTER -> {
        intake.setGoal(Intake.Goal.SHOOTING);
      }
      case AMP_SCORE -> {
        intake.setGoal(Intake.Goal.AMP_SCORING);
      }
      case TRAP_SCORE -> {
        intake.setGoal(Intake.Goal.TRAP_SCORING);
      }
      case SHUFFLE_SHOOTER -> {
        // Shuffle into shooter
        intake.setGoal(Intake.Goal.SHUFFLING);
        if (gamepieceState != GamepieceState.SHOOTER_STAGED) {
          intake.setGoal(Intake.Goal.IDLING);
        }
      }
    }

    intake.periodic();

    // Leds.getInstance().hasNote = gamepieceState != GamepieceState.NONE;
    // Leds.getInstance().intaking = goal == Goal.FLOOR_INTAKE || goal ==
    // Goal.STATION_INTAKE;
  }

  public Command setGoalCommand(Goal goal) {
    return startEnd(() -> this.goal = goal, () -> this.goal = Goal.IDLE)
        .withName("Rollers " + goal);
  }
}
