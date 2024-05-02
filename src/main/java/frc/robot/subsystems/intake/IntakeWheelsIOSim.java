// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class IntakeWheelsIOSim implements IntakeWheelsIO {
  TalonFX leader = new TalonFX(45);
  TalonFXSimState leaderSim = leader.getSimState();
  FlywheelSim flywheelSim;

  public IntakeWheelsIOSim() {
    leaderSim = leader.getSimState();
    flywheelSim = new FlywheelSim(DCMotor.getNeoVortex(1), 4.0, 0.01);
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.Feedback.SensorToMechanismRatio = 3.0;
    leader.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(IntakeWheelsIOInputs inputs) {
    flywheelSim.setInput(leaderSim.getMotorVoltage());
    flywheelSim.update(0.02);
    leaderSim.setRotorVelocity(flywheelSim.getAngularVelocityRadPerSec() / (Math.PI * 2));
    leaderSim.addRotorPosition(0.02 * flywheelSim.getAngularVelocityRadPerSec() / (Math.PI * 2));

    inputs.velocityRPM = leader.getVelocity().refresh().getValue();
    inputs.appliedVolts = leaderSim.getMotorVoltage();
    inputs.currentAmps = new double[] {leaderSim.getSupplyCurrent()};
  }

  @Override
  public void setSpeedRPM(double velocityRPM) {
    leader.setControl(new VelocityVoltage(velocityRPM).withEnableFOC(true));
  }
}
