// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.util.Units;
import frc.robot.VendorWrappers.Neo;

public class ArmIONeo implements ArmIO {

  Neo armMotor;

  public ArmIONeo() {
    armMotor = new Neo(20);

    armMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armRelativePositionDeg = armMotor.getAbsoluteEncoder().getPosition() * Math.PI * 2;
    inputs.armAbsolutePositionDeg = inputs.armRelativePositionDeg;
    inputs.armVelocityRadPerSec = armMotor.getAbsoluteEncoder().getVelocity() * Math.PI * 2;
    inputs.armCurrentAmps = new double[] {armMotor.getOutputCurrent()};
    inputs.armTempCelcius = new double[] {armMotor.getMotorTemperature()};
  }

  @Override
  public void setArmTarget(double target) {
    armMotor.set(Units.radiansToRotations(target));
  }

  @Override
  public void setBrakeMode(boolean armBrake) {
    armMotor.setIdleMode(armBrake ? CANSparkMax.IdleMode.kBrake : CANSparkMax.IdleMode.kCoast);
  }

  @Override
  public void stop() {
    armMotor.stopMotor();
  }
}
