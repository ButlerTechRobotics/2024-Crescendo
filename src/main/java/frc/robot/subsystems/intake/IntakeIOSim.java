// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeIOSim implements IntakeIO {
  FlywheelSim intakeSim;
  double lastAppliedVoltage;

  public IntakeIOSim() {
    intakeSim = new FlywheelSim(DCMotor.getNeoVortex(1), 3.0, 0.01);
    lastAppliedVoltage = 0.0;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.velocityRadPerSec = intakeSim.getAngularVelocityRadPerSec();
    inputs.appliedVolts = lastAppliedVoltage;
    inputs.currentAmps = intakeSim.getCurrentDrawAmps();
  }

  @Override
  public void runVoltage(double voltage) {
    intakeSim.setInputVoltage(voltage);
    lastAppliedVoltage = voltage;
  }
}
