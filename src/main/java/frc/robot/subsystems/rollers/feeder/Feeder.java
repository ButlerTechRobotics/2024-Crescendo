package frc.robot.subsystems.rollers.feeder;

import com.ctre.phoenix.led.CANdle;
import frc.robot.subsystems.rollers.GenericRollerSubsystem;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.RequiredArgsConstructor;
import lombok.Setter;

public class Feeder extends GenericRollerSubsystem<Feeder.Goal> {
  public static CANdle LIGHTS = new CANdle(32);

  @RequiredArgsConstructor
  @Getter
  public enum Goal implements GenericRollerSubsystem.VoltageGoal {
    IDLE(() -> 0.0),
    FLOOR_INTAKING(new LoggedTunableNumber("Feeder/FloorIntakingVoltage", 6.0)),
    BACKSTOPPING(new LoggedTunableNumber("Feeder/BackstoppingVoltage", -3.0)),
    SHOOTING(new LoggedTunableNumber("Feeder/Shooting", 12.0)),
    EJECTING(new LoggedTunableNumber("Feeder/EjectingVoltage", -8.0)),
    EJECTALIGN(new LoggedTunableNumber("Feeder/EjectingAlignVoltage", -1.0)),
    SHOOT(new LoggedTunableNumber("Feeder/ShootVoltage", 12.0)),
    AMPSHOOTER(new LoggedTunableNumber("AmpVoltage", -1.0)),
    AMPSHOOTER2(new LoggedTunableNumber("AmpVoltage2", 6.0));

    private final DoubleSupplier voltageSupplier;
  }

  @Getter @Setter private Feeder.Goal goal = Feeder.Goal.IDLE;

  public Feeder(FeederIO io) {
    super("Feeder", io);
  }
}
