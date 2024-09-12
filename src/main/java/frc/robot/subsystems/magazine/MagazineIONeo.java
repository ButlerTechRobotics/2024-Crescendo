// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.magazine;

import frc.robot.VendorWrappers.Neo;

public class MagazineIONeo implements MagazineIO {
  Neo frontMotor;
  Neo backMotor;

  public MagazineIONeo() {
    frontMotor = new Neo(24);
    backMotor = new Neo(21);

    frontMotor.setSmartCurrentLimit(60);
    backMotor.setSmartCurrentLimit(60);

    frontMotor.setInverted(true);
    backMotor.setInverted(false);
  }

  @Override
  public void updateInputs(MagazineIOInputs inputs) {
    inputs.frontMotorVelocityRadPerSec = frontMotor.getVelocity();
    inputs.frontMotorAppliedVolts = frontMotor.getAppliedOutput() * frontMotor.getBusVoltage();
    inputs.frontMotorCurrentAmps = frontMotor.getOutputCurrent();

    inputs.backMotorVelocityRadPerSec = backMotor.getVelocity();
    inputs.backMotorAppliedVolts = backMotor.getAppliedOutput() * backMotor.getBusVoltage();
    inputs.backMotorCurrentAmps = backMotor.getOutputCurrent();
  }

  public void runVoltage(double frontMotorVoltage, double backMotorVoltage) {
    frontMotor.setVoltage(frontMotorVoltage);
    backMotor.setVoltage(backMotorVoltage);
  }

  @Override
  public void stop() {
    frontMotor.stopMotor();
    backMotor.stopMotor();
  }
}