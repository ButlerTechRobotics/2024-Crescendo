package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.feeder.Feeder;
import frc.robot.subsystems.rollers.intake.Intake;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor()
public class Rollers extends SubsystemBase {
  private final Feeder feeder1;
  private final Feeder feeder2;
  private final Intake intake;

  // Beambreak on DIO 2
  DigitalInput beambreak = new DigitalInput(8);

  public enum Goal {
    IDLE,
    FLOOR_INTAKE,
    STATION_INTAKE,
    EJECT_TO_FLOOR,
    FEED_SHOOTER
  }

  @Getter @Setter private Goal goal = Goal.IDLE;

  @Override
  public void periodic() {

    switch (goal) {
      case IDLE -> {
        feeder1.setGoal(Feeder.Goal.IDLE);
        feeder2.setGoal(Feeder.Goal.IDLE);
        intake.setGoal(Intake.Goal.IDLE);
      }
      case FLOOR_INTAKE -> {
        if (beambreak.get()) {
          feeder1.setGoal(Feeder.Goal.FLOOR_INTAKING);
          feeder2.setGoal(Feeder.Goal.FLOOR_INTAKING);
          intake.setGoal(Intake.Goal.FLOOR_INTAKING);
        } else {
          goal = Goal.IDLE;
          // feeder1.setGoal(Feeder.Goal.IDLE);
          // feeder2.setGoal(Feeder.Goal.IDLE);
          // intake.setGoal(Intake.Goal.IDLE);

          // if (beambreak.get()) {
          //   goal = Goal.IDLE;
          // }
        }
        break;
      }

      case STATION_INTAKE -> {}
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
