package frc.robot.subsystems.rollers.intake;

import frc.robot.subsystems.rollers.GenericRollerSubsystem;
import frc.robot.subsystems.rollers.GenericRollerSubsystem.VoltageGoal;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Intake extends GenericRollerSubsystem<Intake.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements VoltageGoal {
    IDLE(() -> 0.0),
    FLOOR_INTAKING(new LoggedTunableNumber("Intake/FloorIntakingVoltage", 10.0)),
    SHOOTING(new LoggedTunableNumber("Intake/Shooting", 0.0)),
    EJECTING(new LoggedTunableNumber("Intake/EjectingVoltage", -3.0)),
    AMP_SHOOTER(new LoggedTunableNumber("AmpVoltage", -3.0));

    private final DoubleSupplier voltageSupplier;
  }

  @Getter @Setter private Goal goal = Goal.IDLE;

  public Intake(IntakeIO io) {
    super("Intake", io);
  }
}
