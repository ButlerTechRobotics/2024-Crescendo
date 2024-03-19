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
    IDLE(() -> 0.0),
    FLOOR_INTAKING(new LoggedTunableNumber("Intake/FloorIntakingVoltage", 4.0)),
    SHOOTING(new LoggedTunableNumber("Intake/Shooting", 12.0)),
    EJECTING(new LoggedTunableNumber("Intake/EjectingVoltage", -4.0)),
    EJECTALIGN(new LoggedTunableNumber("Intake/EjectingVoltage", -0.03)),
    AMP_SHOOTER(new LoggedTunableNumber("AmpVoltage", 0.0));

    private final DoubleSupplier voltageSupplier;
  }

  private Goal goal = Goal.IDLE;

  public Intake(IntakeIO io) {
    super("Intake", io);
  }
}
