// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.shooter.Shooter;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
public class Superstructure extends SubsystemBase {

  public enum SystemState {
    PREPARE_SHOOT,
    PREPARE_SHOOTMID,
    PREPARE_SHOOTFAR,
    PREPARE_SHOOTTRAP,

    SHOOT,
    SHOOTFAR,
    SHOOTMID,
    SHOOTTRAP,
    PREPARE_INTAKE,
    INTAKE,
    STATION_INTAKE,
    REVERSE_INTAKE,
    IDLE,
  }

  public enum GamepieceState {
    NO_GAMEPIECE,

    HOLDING_SHOOTER
  }

  @Getter private SystemState currentState = SystemState.IDLE;
  @Getter @Setter private SystemState goal = SystemState.IDLE;

  @Getter @Setter private GamepieceState gamepieceState = GamepieceState.NO_GAMEPIECE;

  // private final Arm arm;
  private final Shooter shooter;

  @Override
  public void periodic() {
    switch (goal) {
      case IDLE -> currentState = SystemState.IDLE;
      case STATION_INTAKE -> currentState = SystemState.STATION_INTAKE;
      case INTAKE -> currentState = SystemState.INTAKE;
      case PREPARE_SHOOT -> currentState = SystemState.PREPARE_SHOOT;
      case PREPARE_SHOOTMID -> currentState = SystemState.PREPARE_SHOOTMID;
      case PREPARE_SHOOTFAR -> currentState = SystemState.PREPARE_SHOOTFAR;
      case PREPARE_SHOOTTRAP -> currentState = SystemState.PREPARE_SHOOTTRAP;
      case SHOOT -> currentState = SystemState.SHOOT;
      case SHOOTMID -> currentState = SystemState.SHOOTMID;
      case SHOOTFAR -> currentState = SystemState.SHOOTFAR;
    }

    switch (currentState) {
      case IDLE -> {
        // arm.setSetpoint(Rotation2d.fromDegrees(armIdleSetpointDegrees.get()));
        shooter.setGoal(Shooter.Goal.IDLE);
      }
      case INTAKE -> {
        // arm.setSetpoint(Rotation2d.fromDegrees(armIntakeSetpointDegrees.get()));
        shooter.setGoal(Shooter.Goal.IDLE);
      }
      case STATION_INTAKE -> {
        // arm.setSetpoint(Rotation2d.fromDegrees(armStationIntakeSetpointDegrees.get()));
        shooter.setGoal(Shooter.Goal.INTAKING);
      }
      case REVERSE_INTAKE -> {
        // arm.setSetpoint(Rotation2d.fromDegrees(armIntakeSetpointDegrees.get()));
      }

      case PREPARE_SHOOT -> {
        // arm.setSetpoint(aimingParams.armAngle());
        shooter.setGoal(Shooter.Goal.SHOOTING);
      }

      case PREPARE_SHOOTMID -> {
        // arm.setSetpoint(aimingParams.armAngle());
        shooter.setGoal(Shooter.Goal.SHOOTINGMID);
      }

      case PREPARE_SHOOTFAR -> {
        // arm.setSetpoint(aimingParams.armAngle());
        shooter.setGoal(Shooter.Goal.SHOOTINGFAR);
      }

      case PREPARE_SHOOTTRAP -> {
        // arm.setSetpoint(aimingParams.armAngle());
        shooter.setGoal(Shooter.Goal.SHOOTINGTRAP);
      }

      case SHOOT -> {
        shooter.setGoal(Shooter.Goal.SHOOTING);
        // arm.setSetpoint(aimingParams.armAngle());
      }
      case SHOOTMID -> {
        shooter.setGoal(Shooter.Goal.SHOOTINGMID);
        // arm.setSetpoint(aimingParams.armAngle());
      }
      case SHOOTFAR -> {
        shooter.setGoal(Shooter.Goal.SHOOTINGFAR);
        // arm.setSetpoint(aimingParams.armAngle());
      }
      case SHOOTTRAP -> {
        shooter.setGoal(Shooter.Goal.SHOOTINGTRAP);
        // arm.setSetpoint(aimingParams.armAngle());
      }
    }

    Logger.recordOutput("Superstructure/GoalState", goal);
    Logger.recordOutput("Superstructure/CurrentState", currentState);
  }

  @AutoLogOutput(key = "Superstructure/ReadyToShoot")
  public boolean atShootingSetpoint() {
    return shooter.atSetpoint();
  }
}
