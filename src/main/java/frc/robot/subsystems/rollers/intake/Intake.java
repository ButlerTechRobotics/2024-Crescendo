// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.intake;

import frc.robot.subsystems.rollers.GenericRollerSystem;
import frc.robot.subsystems.rollers.GenericRollerSystem.VoltageGoal;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

@Setter
@Getter
public class Intake extends GenericRollerSystem<Intake.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements VoltageGoal {
    IDLING(() -> 0.0),
    FLOOR_INTAKING(new LoggedTunableNumber("Intake/FloorIntakingVoltage", 10.0)),
    EJECTING(new LoggedTunableNumber("Intake/EjectingVoltage", -8.0)),
    SHOOTING(new LoggedTunableNumber("Intake/ShootingVoltage", 12.0)),
    AMP_SCORING(new LoggedTunableNumber("Intake/AmpVoltage", 5.0)),
    TRAP_SCORING(new LoggedTunableNumber("Intake/TrapVoltage", 5.0)),
    SHUFFLING(new LoggedTunableNumber("Intake/ShufflingVoltage", -1.0));

    private final DoubleSupplier voltageSupplier;
  }

  private Goal goal = Goal.IDLING;

  public Intake(IntakeIO io) {
    super("Intake", io);
  }
}
