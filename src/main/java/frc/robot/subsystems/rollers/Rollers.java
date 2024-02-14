package frc.robot.subsystems.rollers;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public boolean getBeamBreak() {
    return beambreak.get();
  }

  public enum Goal {
    IDLE,
    FLOOR_INTAKE,
    EJECT_TO_FLOOR,
    FEED_SHOOTER,
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
        feeder1.setGoal(Feeder.Goal.FLOOR_INTAKING);
        feeder2.setGoal(Feeder.Goal.FLOOR_INTAKING);
        intake.setGoal(Intake.Goal.FLOOR_INTAKING);
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
