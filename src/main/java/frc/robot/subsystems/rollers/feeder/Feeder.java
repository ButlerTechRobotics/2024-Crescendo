// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers.feeder;

import frc.robot.subsystems.rollers.GenericRollerSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Feeder extends GenericRollerSubsystem<Feeder.Goal> {
  @RequiredArgsConstructor
  @Getter
  public enum Goal implements GenericRollerSubsystem.VoltageGoal {
    IDLE(() -> 0.0),
    FLOOR_INTAKING(new LoggedTunableNumber("Feeder/FloorIntakingVoltage", 6.0)),
    BACKSTOPPING(new LoggedTunableNumber("Feeder/BackstoppingVoltage", -3.0)),
    SHOOTING(new LoggedTunableNumber("Feeder/Shooting", 12.0)),
    EJECTING(new LoggedTunableNumber("Feeder/EjectingVoltage", -6.0)),
    EJECTALIGN(new LoggedTunableNumber("Feeder/EjectingAlignVoltage", -1.0)),
    SHOOT(new LoggedTunableNumber("Feeder/ShootVoltage", 12.0)),
    AMPSHOOTER(new LoggedTunableNumber("AmpVoltage", -3)),
    AMPSHOOTER2(new LoggedTunableNumber("Amp2Voltage", -12));

    private final DoubleSupplier voltageSupplier;
  }

  @Getter @Setter private Feeder.Goal goal = Feeder.Goal.IDLE;

  public Feeder(FeederIO io) {
    super("Feeder", io);
  }
}
