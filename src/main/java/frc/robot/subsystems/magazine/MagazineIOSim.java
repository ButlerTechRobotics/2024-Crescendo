// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.magazine;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class MagazineIOSim implements MagazineIO {

  FlywheelSim frontMotorSim;
  FlywheelSim backMotorSim;

  double frontLastAppliedVoltage;
  double backLastAppliedVoltage;

  public MagazineIOSim() {
    frontMotorSim = new FlywheelSim(DCMotor.getNeoVortex(1), 1.0, 0.01);
    backMotorSim = new FlywheelSim(DCMotor.getNeoVortex(1), 1.0, 0.01);
    frontLastAppliedVoltage = 0.0;
    backLastAppliedVoltage = 0.0;
  }

  @Override
  public void updateInputs(MagazineIOInputs inputs) {
    inputs.frontMotorVelocityRadPerSec = frontMotorSim.getAngularVelocityRadPerSec();
    inputs.frontMotorAppliedVolts = frontLastAppliedVoltage;
    inputs.frontMotorCurrentAmps = frontMotorSim.getCurrentDrawAmps();

    inputs.backMotorVelocityRadPerSec = backMotorSim.getAngularVelocityRadPerSec();
    inputs.backMotorAppliedVolts = backLastAppliedVoltage;
    inputs.backMotorCurrentAmps = backMotorSim.getCurrentDrawAmps();
  }

  public void runVoltage(double frontMotorVoltage, double backMotorVoltage) {
    frontMotorSim.setInputVoltage(frontMotorVoltage);
    backMotorSim.setInputVoltage(backMotorVoltage);

    frontLastAppliedVoltage = frontMotorVoltage;
    backLastAppliedVoltage = backMotorVoltage;
  }
}
