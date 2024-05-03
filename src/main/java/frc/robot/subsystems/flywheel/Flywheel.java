// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheel;

import static frc.robot.subsystems.flywheel.FlywheelConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheels/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Flywheels/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheels/kD", gains.kD());
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheels/kS", gains.kS());
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheels/kV", gains.kV());
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheels/kA", gains.kA());

  private static final LoggedTunableNumber flywheelTolerance =
      new LoggedTunableNumber("Flywheels/ToleranceRPM", flywheelToleranceRPM);

  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

  private Double topSetpointRpm = null;
  private Double bottomSetpointRPM = null;

  public void setSetpoint(double top, double bottom) {
    topSetpointRpm = top + 100;
    bottomSetpointRPM = bottom;
    io.runVelocity(top, bottom);
  }

  public void stop() {
    topSetpointRpm = null;
    bottomSetpointRPM = null;
    io.stop();
  }

  public Flywheel(FlywheelIO io) {
    System.out.println("[Init] Creating flywheel");
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

  @AutoLogOutput(key = "flywheel/AtSetpoint")
  public boolean atSetpoint() {
    return topSetpointRpm != null
        && bottomSetpointRPM != null
        && Math.abs(inputs.topVelocityRpm - topSetpointRpm) <= flywheelTolerance.get()
        && Math.abs(inputs.bottomVelocityRpm - bottomSetpointRPM) <= flywheelTolerance.get();
  }
}
