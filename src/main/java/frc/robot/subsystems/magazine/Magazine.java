// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.magazine;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Magazine extends SubsystemBase {
  private final MagazineIO magazineIO;
  private final MagazineIOInputsAutoLogged inputs = new MagazineIOInputsAutoLogged();
  private double frontTargetVoltage = 0;
  private double backTargetVoltage = 0;
  private boolean intakeRequest = false;
  private boolean outtakeRequest = false;

  /** Creates a new Magazine. */
  public Magazine(MagazineIO magazineIO) {
    this.magazineIO = magazineIO;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    magazineIO.updateInputs(inputs);
    Logger.processInputs("Magazine", inputs);
  }

  public void intake() {
    setVoltage(6, 6);
  }

  public void shoot() {
    setVoltage(12, 12);
  }

  public void outtake() {
    setVoltage(-12, -2);
  }

  public void slowForward() {
    setVoltage(3, 3);
  }

  public void slowBackward() {
    setVoltage(-1, -1);
  }

  public void stop() {
    magazineIO.stop();
    setVoltage(0, 0);
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

  public void setVoltage(double frontMotorVoltage, double backMotorVoltage) {
    frontTargetVoltage = frontMotorVoltage;
    backTargetVoltage = backMotorVoltage;
    magazineIO.runVoltage(frontTargetVoltage, backTargetVoltage);
  }

  @AutoLogOutput(key = "Intake/IntakeRequest")
  public boolean getIntakeRequest() {
    return intakeRequest;
  }

  @AutoLogOutput(key = "Intake/OuttakeRequest")
  public boolean getOuttakeRequest() {
    return outtakeRequest;
  }

  @AutoLogOutput(key = "Magazine/FrontTargetVoltage")
  public double getFrontTargetVoltage() {
    return frontTargetVoltage;
  }

  @AutoLogOutput(key = "Magazine/BackTargetVoltage")
  public double getBackTargetVoltage() {
    return backTargetVoltage;
  }

  @AutoLogOutput(key = "Magazine/FrontMotorVelocityRadPerSec")
  public double frontVelocityRadPerSec() {
    return inputs.frontMotorVelocityRadPerSec;
  }

  @AutoLogOutput(key = "Magazine/BackMotorVelocityRadPerSec")
  public double backVelocityRadPerSec() {
    return inputs.backMotorVelocityRadPerSec;
  }
}
