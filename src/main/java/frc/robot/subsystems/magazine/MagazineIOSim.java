// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.magazine;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class MagazineIOSim implements MagazineIO {
  TalonFX topMotor = new TalonFX(18);
  TalonFX bottomMotor = new TalonFX(19);

  TalonFXSimState topMotorSim = topMotor.getSimState();
  TalonFXSimState bottomMotorSim = bottomMotor.getSimState();

  FlywheelSim topMotorMagazineSIM;
  FlywheelSim bottomMotorMagazineSIM;

  public MagazineIOSim() {
    topMotorSim = topMotor.getSimState();
    topMotorMagazineSIM = new FlywheelSim(DCMotor.getNeoVortex(1), 3.0, 0.01);
    TalonFXConfiguration topConfig = new TalonFXConfiguration();
    var slot0Configs = topConfig.Slot0;
    slot0Configs.kP = 9.2;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
    topConfig.Feedback.SensorToMechanismRatio = 3.0;
    topMotor.getConfigurator().apply(topConfig);

    bottomMotorSim = bottomMotor.getSimState();
    bottomMotorMagazineSIM = new FlywheelSim(DCMotor.getNeoVortex(1), 3.0, 0.01);
    TalonFXConfiguration bottomConfig = new TalonFXConfiguration();
    var slot1Configs = bottomConfig.Slot0;
    slot1Configs.kP = 9.2;
    slot1Configs.kI = 0;
    slot1Configs.kD = 0;
    bottomConfig.Feedback.SensorToMechanismRatio = 3.0;
    topMotor.getConfigurator().apply(bottomConfig);
  }

  @Override
  public void updateInputs(MagazineIOInputs inputs) {
    topMotorMagazineSIM.setInput(topMotorSim.getMotorVoltage());
    topMotorMagazineSIM.update(0.02);
    topMotorSim.setRotorVelocity(topMotorMagazineSIM.getAngularVelocityRadPerSec() / (Math.PI * 2));
    topMotorSim.addRotorPosition(
        0.02 * topMotorMagazineSIM.getAngularVelocityRadPerSec() / (Math.PI * 2));

    inputs.topVelocityRPM = topMotor.getVelocity().refresh().getValue() * (Math.PI * 2.0);
    inputs.topVelocityRPM = topMotor.getVelocity().refresh().getValue() * (Math.PI * 2.0);

    inputs.topAppliedVolts = topMotorSim.getMotorVoltage();
    inputs.topCurrentAmps = new double[] {topMotorSim.getSupplyCurrent()};

    bottomMotorMagazineSIM.setInput(bottomMotorSim.getMotorVoltage());
    bottomMotorMagazineSIM.update(0.02);
    bottomMotorSim.setRotorVelocity(
        bottomMotorMagazineSIM.getAngularVelocityRadPerSec() / (Math.PI * 2));
    bottomMotorSim.addRotorPosition(
        0.02 * bottomMotorMagazineSIM.getAngularVelocityRadPerSec() / (Math.PI * 2));

    inputs.bottomVelocityRPM = bottomMotor.getVelocity().refresh().getValue() * (Math.PI * 2.0);
    inputs.bottomVelocityRPM = bottomMotor.getVelocity().refresh().getValue() * (Math.PI * 2.0);

    inputs.bottomAppliedVolts = bottomMotorSim.getMotorVoltage();
    inputs.bottomCurrentAmps = new double[] {bottomMotorSim.getSupplyCurrent()};
  }

  @Override
  public void runVoltage(double topVoltage, double bottomVoltage) {
    topMotor.setControl(new VoltageOut(topVoltage));
    bottomMotor.setControl(new VoltageOut(bottomVoltage));
  }
}
