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
  private static final LoggedTunableNumber topkP =
      new LoggedTunableNumber("Flywheels/topkP", gains.topkP());
  private static final LoggedTunableNumber bottomkP =
      new LoggedTunableNumber("Flywheels/bottomkP", gains.bottomkP());
  private static final LoggedTunableNumber topkS =
      new LoggedTunableNumber("Flywheels/topkS", gains.topkS());
  private static final LoggedTunableNumber topkV =
      new LoggedTunableNumber("Flywheels/topkV", gains.topkV());
  private static final LoggedTunableNumber topkA =
      new LoggedTunableNumber("Flywheels/topkA", gains.topkA());
  private static final LoggedTunableNumber bottomkS =
      new LoggedTunableNumber("Flywheels/bottomkS", gains.bottomkS());
  private static final LoggedTunableNumber bottomkV =
      new LoggedTunableNumber("Flywheels/bottomkV", gains.bottomkV());
  private static final LoggedTunableNumber bottomkA =
      new LoggedTunableNumber("Flywheels/bottomkA", gains.bottomkA());

  private static final LoggedTunableNumber shooterTolerance =
      new LoggedTunableNumber("Flywheels/ToleranceRPM", shooterToleranceRPM);

  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private Double topSetpointRpm = null;
  private Double bottomSetpointRPM = null;

  public void setSetpoint(double top, double bottom) {
    // topSetpointRpm = top + 100;
    // bottomSetpointRPM - bottom;
    topSetpointRpm = top;
    bottomSetpointRPM = bottom;
    io.runVelocity(top, bottom);
  }

  public void manualBlurp() {
    topSetpointRpm = 3000.0;
    bottomSetpointRPM = 3000.0;
    io.runVelocity(topSetpointRpm, bottomSetpointRPM);
  }

  public void stop() {
    topSetpointRpm = 0.0;
    bottomSetpointRPM = 0.0;
    io.stop();
  }

  public Shooter(ShooterIO io) {
    System.out.println("[Init] Creating Shooter");
    this.io = io;
  }

  @Override
  public void periodic() {
    // check controllers
    LoggedTunableNumber.ifChanged(hashCode(), pp -> io.setPID(pp[0], pp[1]), topkP, bottomkP);
    LoggedTunableNumber.ifChanged(
        hashCode(),
        kSVASVA -> io.setFF(kSVASVA[0], kSVASVA[1], kSVASVA[2], kSVASVA[3], kSVASVA[4], kSVASVA[5]),
        topkS,
        topkV,
        topkA,
        bottomkS,
        bottomkV,
        bottomkA);

    io.updateInputs(inputs);
    Logger.processInputs("Flywheels", inputs);

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
  public boolean atTargetSpeed() {
    return topSetpointRpm != null
        && bottomSetpointRPM != null
        && Math.abs(inputs.topVelocityRpm - topSetpointRpm) <= shooterTolerance.get()
        && Math.abs(inputs.bottomVelocityRpm - bottomSetpointRPM) <= shooterTolerance.get();
  }
}
