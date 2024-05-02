// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.VendorWrappers.Neo;

public class IntakeWheelsIONeo implements IntakeWheelsIO {

  Neo leader;
  PIDController controller = new PIDController(0, 0, 0);

  public IntakeWheelsIONeo() {
    leader = new Neo(25);
    leader.setSmartCurrentLimit(30);
    controller.setPID(0.012, 0, 0);
  }

  @Override
  public void updateInputs(IntakeWheelsIOInputs inputs) {
    inputs.velocityRPM = leader.getEncoder().getVelocity();
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
  }

  @Override
  public void setSpeedRPM(double velocityRPM) {
    leader.getPIDController().setReference(velocityRPM, ControlType.kVelocity);
  }

  @Override
  public void setVotSpeed(double appliedVolts) {
    leader.setVoltage(appliedVolts);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
