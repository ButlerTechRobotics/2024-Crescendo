// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.feeder.Feeder;
import frc.robot.subsystems.rollers.intake.Intake;
// import frc.robot.subsystems.superstructure.arm.ArmPositionPID;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor()
public class Rollers extends SubsystemBase {
  // private final ArmPositionPID armPID;
  private final Feeder feeder1;
  private final Feeder feeder2;
  private final Intake intake;

  // Beambreak on DIO 2
  DigitalInput beambreak = new DigitalInput(9);

  public boolean getBeamBreak() {
    return beambreak.get();
  }

  public enum Goal {
    IDLE,
    FLOOR_INTAKE,
    EJECT_TO_FLOOR,
    FEED_SHOOTER,
    AMP_SHOOTER,
    EJECTALIGN,
    SHOOT
  }

  @Getter @Setter private Goal goal = Goal.IDLE;

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("BeamBreak", beambreak.get());

    switch (goal) {
      case IDLE -> {
        feeder1.setGoal(Feeder.Goal.IDLE);
        feeder2.setGoal(Feeder.Goal.IDLE);
        intake.setGoal(Intake.Goal.IDLE);
      }
      case FLOOR_INTAKE -> {
        // if (armPID.isAtHomePosition()) {
        feeder1.setGoal(Feeder.Goal.FLOOR_INTAKING);
        feeder2.setGoal(Feeder.Goal.FLOOR_INTAKING);
        intake.setGoal(Intake.Goal.FLOOR_INTAKING);
        // } else {
        //   // The arm is not at its home position, so set the goals to IDLE
        //   feeder1.setGoal(Feeder.Goal.IDLE);
        //   feeder2.setGoal(Feeder.Goal.IDLE);
        //   intake.setGoal(Intake.Goal.IDLE);
        // }
      }

      case SHOOT -> {
        feeder1.setGoal(Feeder.Goal.SHOOT);
        feeder2.setGoal(Feeder.Goal.SHOOT);
      }

      case EJECTALIGN -> {
        feeder1.setGoal(Feeder.Goal.EJECTALIGN);
        feeder2.setGoal(Feeder.Goal.EJECTALIGN);
        intake.setGoal(Intake.Goal.IDLE);
      }

      case AMP_SHOOTER -> {
        feeder1.setGoal(Feeder.Goal.AMPSHOOTER2);
        feeder2.setGoal(Feeder.Goal.AMPSHOOTER);
      }

      case FEED_SHOOTER -> {
        intake.setGoal(Intake.Goal.SHOOTING);
        feeder1.setGoal(Feeder.Goal.SHOOTING);
        feeder2.setGoal(Feeder.Goal.SHOOTING);
      }
      case EJECT_TO_FLOOR -> {
        feeder1.setGoal(Feeder.Goal.EJECTING);
        feeder2.setGoal(Feeder.Goal.EJECTING);
        intake.setGoal(Intake.Goal.EJECTING);
      }
    }

    Logger.recordOutput("Rollers/Goal", goal);

    feeder1.periodic();
    feeder2.periodic();
    intake.periodic();
  }
}
