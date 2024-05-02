// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.magazine;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.VendorWrappers.Neo;

public class MagazineIONeo implements MagazineIO {
  Neo topMotor;
  Neo bottomMotor;

  PIDController controller = new PIDController(0, 0, 0);

  public MagazineIONeo() {
    topMotor = new Neo(21);
    bottomMotor = new Neo(24);

    topMotor.setSmartCurrentLimit(30);
    bottomMotor.setSmartCurrentLimit(30);
    controller.setPID(0.012, 0, 0);
  }

  @Override
  public void updateInputs(MagazineIOInputs inputs) {
    inputs.topVelocityRPM = topMotor.getEncoder().getVelocity();
    inputs.bottomVelocityRPM = bottomMotor.getEncoder().getVelocity();

    inputs.topAppliedVolts = topMotor.getAppliedOutput() * topMotor.getBusVoltage();
    inputs.bottomAppliedVolts = bottomMotor.getAppliedOutput() * bottomMotor.getBusVoltage();

    inputs.topCurrentAmps = new double[] {topMotor.getOutputCurrent()};
    inputs.bottomCurrentAmps = new double[] {bottomMotor.getOutputCurrent()};
  }

  public void runVoltage(double topVoltage, double bottomVoltage) {
    topMotor.setVoltage(topVoltage);
    bottomMotor.setVoltage(bottomVoltage);
  }

  @Override
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }
}
