package frc.robot.subsystems.superstructure.shooter;

import static frc.robot.subsystems.superstructure.SuperstructureConstants.ShooterConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheels/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", gains.kD());
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheels/kS", gains.kS());
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", gains.kV());
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", gains.kA());

  private static LoggedTunableNumber shootingTopRPM =
      new LoggedTunableNumber("Superstructure/ShootingTopRPM", 6000.0);
  private static LoggedTunableNumber shootingBottomRPM =
      new LoggedTunableNumber("Superstructure/ShootingBottomRPM", 6000.0);

  private static LoggedTunableNumber intakingTopRPM =
      new LoggedTunableNumber("Superstructure/IntakingTopRPM", -2000.0);
  private static LoggedTunableNumber intakingBottomRPM =
      new LoggedTunableNumber("Superstructure/IntakingBottomRPM", -2000.0);

  private static LoggedTunableNumber idleTopRPM =
      new LoggedTunableNumber("Superstructure/IdleTopRPM", 200.0);
  private static LoggedTunableNumber idleBottomRPM =
      new LoggedTunableNumber("Superstructure/IdleBottomRPM", 200.0);

  private static final LoggedTunableNumber shooterTolerance =
      new LoggedTunableNumber("Flywheels/ToleranceRPM", shooterToleranceRPM);

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private Double topSetpointRpm = null;
  private Double bottomSetpointRPM = null;

  public enum Goal {
    IDLE,
    SHOOTING,
    INTAKING,

    CHARACTERIZATION
  }

  @Getter @Setter private Goal goal = Goal.IDLE;

  public void setSetpoint(double top, double bottom) {
    topSetpointRpm = top;
    bottomSetpointRPM = bottom;
    io.runVelocity(top, bottom);
  }

  public void stop() {
    topSetpointRpm = null;
    bottomSetpointRPM = null;
    io.stop();
  }

  public Shooter(ShooterIO io) {
    System.out.println("[Init] Creating Shooter");
    this.io = io;
  }

  @Override
  public void periodic() {
    // check controllers
    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), kSVA -> io.setFF(kSVA[0], kSVA[1], kSVA[2]), kS, kV, kA);

    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);

    if (DriverStation.isDisabled()) {
      stop();
      setGoal(Goal.IDLE);
    } else {
      switch (goal) {
        case IDLE -> setSetpoint(idleTopRPM.get(), idleBottomRPM.get());
        case INTAKING -> setSetpoint(intakingTopRPM.get(), intakingBottomRPM.get());
        case SHOOTING -> setSetpoint(shootingTopRPM.get(), shootingBottomRPM.get());
      }
    }

    Logger.recordOutput("Flywheels/Goal", goal);
    Logger.recordOutput("Flywheels/TopSetpointRPM", topSetpointRpm != null ? topSetpointRpm : 0.0);
    Logger.recordOutput(
        "Flywheels/BottomSetpointRPM", bottomSetpointRPM != null ? bottomSetpointRPM : 0.0);
    Logger.recordOutput("Flywheels/TopRPM", inputs.topVelocityRpm);
    Logger.recordOutput("Flywheels/BottomRPM", inputs.bottomVelocityRpm);
  }

  public void runTopCharacterizationVolts(double volts) {
    io.runCharacterizationTopVolts(volts);
  }

  public void runBottomCharacterizationVolts(double volts) {
    io.runCharacterizationBottomVolts(volts);
  }

  public double getTopCharacterizationVelocity() {
    return inputs.topVelocityRpm;
  }

  public double getBottomCharacterizationVelocity() {
    return inputs.bottomVelocityRpm;
  }

  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public boolean atSetpoint() {
    return topSetpointRpm != null
        && bottomSetpointRPM != null
        && Math.abs(inputs.topVelocityRpm - topSetpointRpm) <= shooterTolerance.get()
        && Math.abs(inputs.bottomVelocityRpm - bottomSetpointRPM) <= shooterTolerance.get();
  }
}
