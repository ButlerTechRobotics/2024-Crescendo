// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheels/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", gains.kD());
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheels/kS", gains.kS());
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", gains.kV());
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", gains.kA());

  private static final LoggedTunableNumber shooterTolerance =
      new LoggedTunableNumber("Flywheels/ToleranceRPM", shooterToleranceRPM);

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private Double topSetpointRPM = null;
  private Double bottomSetpointRPM = null;

  private double topTargetRPM = 0.0;
  private double bottomTargetRPM = 0.0;

  public void setSetpoint(double top, double bottom) {
    topSetpointRPM = top + 100;
    bottomSetpointRPM = bottom;
    io.runVelocity(top, bottom);
  }

  public void stop() {
    topSetpointRPM = null;
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

    Logger.recordOutput("Flywheels/TopSetpointRPM", topSetpointRPM != null ? topSetpointRPM : 0.0);
    Logger.recordOutput(
        "Flywheels/BottomSetpointRPM", bottomSetpointRPM != null ? bottomSetpointRPM : 0.0);
    Logger.recordOutput("Flywheels/TopRPM", inputs.topVelocityRPM);
    Logger.recordOutput("Flywheels/BottomRPM", inputs.bottomVelocityRPM);
  }

  public void runTopCharacterizationVolts(double volts) {
    io.runCharacterizationTopVolts(volts);
  }

  public void runBottomCharacterizationVolts(double volts) {
    io.runCharacterizationBottomVolts(volts);
  }

  public double getTopCharacterizationVelocity() {
    return inputs.topVelocityRPM;
  }

  public double getBottomCharacterizationVelocity() {
    return inputs.bottomVelocityRPM;
  }

  @AutoLogOutput(key = "Flywheel/TopTargetRPM")
  public double getTopTargetRPM() {
    return topTargetRPM;
  }

  @AutoLogOutput(key = "Flywheel/BottomTargetRPM")
  public double getBottomTargetRPM() {
    return bottomTargetRPM;
  }

  public boolean atTargetSpeed() {
    return Math.abs(inputs.topVelocityRPM - getTopTargetRPM()) < 1
        && Math.abs(inputs.bottomVelocityRPM - getBottomTargetRPM()) < 1;
  }
}
