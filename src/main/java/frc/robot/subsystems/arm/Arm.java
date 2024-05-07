// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Arm/kP", gains.kP());
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Arm/kI", gains.kI());
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Arm/kD", gains.kD());
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Arm/kS", gains.kS());
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Arm/kV", gains.kV());
  private static final LoggedTunableNumber kA = new LoggedTunableNumber("Arm/kA", gains.kA());

  private static final LoggedTunableNumber armTolerance =
      new LoggedTunableNumber("Arm/ToleranceDeg", armToleranceDeg);

  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  private Double armSetpointDeg = null;

  public void setArmTargetAngle(double angle) {
    armSetpointDeg = angle;
    io.runPosition(angle);
  }

  public void stop() {
    armSetpointDeg = 0.0;
    io.stop();
  }

  public Arm(ArmIO io) {
    System.out.println("[Init] Creating Arm");
    this.io = io;
  }

  @Override
  public void periodic() {
    // check controllers
    LoggedTunableNumber.ifChanged(hashCode(), pid -> io.setPID(pid[0], pid[1], pid[2]), kP, kI, kD);
    LoggedTunableNumber.ifChanged(
        hashCode(), kSVA -> io.setFF(kSVA[0], kSVA[1], kSVA[2]), kS, kV, kA);

    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);

    Logger.recordOutput("Arm/SetAngle", armSetpointDeg != null ? armSetpointDeg : 0.0);
    Logger.recordOutput("Arm/CurrentAngle", inputs.armCurrentAngleDeg);
  }

  public void runArmCharacterizationVolts(double volts) {
    io.runCharacterizationArmVolts(volts);
  }

  public double getArmCurrentAngle() {
    return inputs.armCurrentAngleDeg;
  }

  public double getArmTargetAngle() {
    return armSetpointDeg;
  }

  @AutoLogOutput(key = "Arm/AtSetpoint")
  public boolean atSetpoint() {
    return armSetpointDeg != null
        && Math.abs(inputs.armCurrentAngleDeg - armSetpointDeg) <= armTolerance.get();
  }
}
