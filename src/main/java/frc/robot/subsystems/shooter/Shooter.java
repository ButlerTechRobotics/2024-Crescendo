package frc.robot.subsystems.shooter;

import static frc.robot.Constants.ShooterConstants.*;

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
      new LoggedTunableNumber("Shooter/ShootingTopRPM", 3500.0);
  private static LoggedTunableNumber shootingBottomRPM =
      new LoggedTunableNumber("Shooter/ShootingBottomRPM", 3500.0);

  private static final LoggedTunableNumber shooterTolerance =
      new LoggedTunableNumber("Shooter/ToleranceRPM", shooterToleranceRPM);

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
    Logger.processInputs("Shooter", inputs);

    if (DriverStation.isDisabled()) {
      stop();
      setGoal(Goal.IDLE);
    } else {
      switch (goal) {
        case IDLE -> io.stop();
        case SHOOTING -> setSetpoint(shootingTopRPM.get(), shootingBottomRPM.get());
      }
    }

    Logger.recordOutput("Shooter/Goal", goal);
    Logger.recordOutput("Shooter/TopSetpointRPM", topSetpointRpm != null ? topSetpointRpm : 0.0);
    Logger.recordOutput(
        "Shooter/BottomSetpointRPM", bottomSetpointRPM != null ? bottomSetpointRPM : 0.0);
    Logger.recordOutput("Shooter/TopRPM", inputs.topVelocityRpm);
    Logger.recordOutput("Shooter/BottomRPM", inputs.bottomVelocityRpm);
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

  /**
     * Returns true if both flywheels are spinning within some threshold of their target speeds.
     * @param rpmThreshold - Max distance from the setpoint for this function to still return true, in meters per second.
     */
    public boolean flywheelsAtSetpoints(double topVelocityRpm, double bottomVelocityRpm, double thresholdRPM) {

        return 
            Math.abs(topVelocityRpm - inputs.topVelocityRpm) < thresholdRPM
                && Math.abs(bottomVelocityRpm - inputs.bottomVelocityRpm) < thresholdRPM;
    }

  @AutoLogOutput(key = "Shooter/AtSetpoint")
  public boolean atSetpoint() {
    return topSetpointRpm != null
        && bottomSetpointRPM != null
        && Math.abs(inputs.topVelocityRpm - topSetpointRpm) <= shooterTolerance.get()
        && Math.abs(inputs.bottomVelocityRpm - bottomSetpointRPM) <= shooterTolerance.get();
  }
}