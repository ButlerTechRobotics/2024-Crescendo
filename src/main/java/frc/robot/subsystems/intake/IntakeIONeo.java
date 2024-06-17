// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import frc.robot.VendorWrappers.Neo;

public class IntakeIONeo implements IntakeIO {
  Neo intakeMotor;

  public IntakeIONeo() {
    intakeMotor = new Neo(25);
    intakeMotor.setSmartCurrentLimit(30);
    intakeMotor.setInverted(true);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRadPerSec = intakeMotor.getVelocity();
    inputs.appliedVolts = intakeMotor.getAppliedOutput() * intakeMotor.getBusVoltage();
    inputs.currentAmps = intakeMotor.getOutputCurrent();
  }

  public void runVoltage(double voltage) {
    intakeMotor.setVoltage(voltage);
  }

  @Override
  public void stop() {
    intakeMotor.stopMotor();
  }
}
