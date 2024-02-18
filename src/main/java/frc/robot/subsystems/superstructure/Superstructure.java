package frc.robot.subsystems.superstructure;

// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.subsystems.superstructure.arm.Arm;
import frc.robot.subsystems.superstructure.shooter.Shooter;
// import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

@RequiredArgsConstructor
public class Superstructure extends SubsystemBase {
  // private static LoggedTunableNumber armIdleSetpointDegrees =
  //     new LoggedTunableNumber("Superstructure/ArmIdleSetpointDegrees", 20.0);
  // private static LoggedTunableNumber armStationIntakeSetpointDegrees =
  //     new LoggedTunableNumber("Superstructure/ArmStationIntakeSetpointDegrees", 45.0);
  // private static LoggedTunableNumber armIntakeSetpointDegrees =
  //     new LoggedTunableNumber("Superstructure/ArmIntakeDegrees", 40.0);
  // private static LoggedTunableNumber followThroughTime =
  //     new LoggedTunableNumber("Superstructure/FollowthroughTimeSecs", 0.5);

  public enum SystemState {
    PREPARE_SHOOT,
    SHOOT,
    PREPARE_INTAKE,
    INTAKE,
    STATION_INTAKE,
    REVERSE_INTAKE,
    IDLE,
  }

  public enum GamepieceState {
    NO_GAMEPIECE,

    HOLDING_SHOOTER,

    HOLDING_BACKPACK
  }

  @Getter private SystemState currentState = SystemState.IDLE;
  @Getter @Setter private SystemState goal = SystemState.IDLE;

  @Getter @Setter private GamepieceState gamepieceState = GamepieceState.NO_GAMEPIECE;

  // private final Arm arm;
  private final Shooter shooter;

  // private final Timer followThroughTimer = new Timer();

  @Override
  public void periodic() {
    switch (goal) {
      case IDLE -> currentState = SystemState.IDLE;
      case STATION_INTAKE -> currentState = SystemState.STATION_INTAKE;
      case INTAKE -> currentState = SystemState.INTAKE;
      case PREPARE_SHOOT -> currentState = SystemState.PREPARE_SHOOT;
      case SHOOT -> currentState = SystemState.SHOOT;
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
        // var aimingParams = RobotState.getInstance().getAimingParameters();
        // arm.setSetpoint(aimingParams.armAngle());
        shooter.setGoal(Shooter.Goal.SHOOTING);
      }
      case SHOOT -> {
        // var aimingParams = RobotState.getInstance().getAimingParameters();
        shooter.setGoal(Shooter.Goal.SHOOTING);
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
