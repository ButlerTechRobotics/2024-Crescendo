// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO intakeIO;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();
  private double targetVoltage = 0;
  private boolean intakeRequest = false;
  private boolean outtakeRequest = false;

  /** Creates a new Intake. */
  public Intake(IntakeIO intakeIO) {
    this.intakeIO = intakeIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    intakeIO.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void intake() {
    setVoltage(12);
  }

  public void outtake() {
    setVoltage(-12);
  }

  public void stop() {
    intakeIO.stop();
    setVoltage(0);
  }

  public void enableIntakeRequest() {
    this.intakeRequest = true;
  }

  public void disableIntakeRequest() {
    this.intakeRequest = false;
  }

  public void enableOuttakeRequest() {
    this.outtakeRequest = true;
  }

  public void disableOuttakeRequest() {
    this.outtakeRequest = false;
  }

  public void setVoltage(double voltage) {
    targetVoltage = voltage;
    intakeIO.runVoltage(targetVoltage);
  }

  @AutoLogOutput(key = "Intake/IntakeRequest")
  public boolean getIntakeRequest() {
    return intakeRequest;
  }

  @AutoLogOutput(key = "Intake/OuttakeRequest")
  public boolean getOuttakeRequest() {
    return outtakeRequest;
  }

  @AutoLogOutput(key = "Intake/TargetVoltage")
  public double getTargetVoltage() {
    return targetVoltage;
  }

  @AutoLogOutput(key = "Intake/VelocityRadPerSec")
  public double velocityRadsPerSec() {
    // Convert Radians per second to Meters per second
    return inputs.velocityRadPerSec;
  }
}
