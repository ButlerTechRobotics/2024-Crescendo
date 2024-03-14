// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

public class ArmPDController {

  double kP, kD;
  double pSetpoint, dSetpoint;

  public ArmPDController(double kP, double kD) {
    this.kP = kP;
    this.kD = kD;
  }

  public void setSetpoint(double pSetpoint, double dSetpoint) {
    this.pSetpoint = pSetpoint;
    this.dSetpoint = dSetpoint;
  }

  public double calculate(double pMeasurement, double dMeasurement) {
    return kP * (pSetpoint - pMeasurement) + kD * (dSetpoint - dMeasurement);
  }

  public double calculate(
      double pMeasurement, double dMeasurement, double pSetpoint, double dSetpoint) {
    this.pSetpoint = pSetpoint;
    this.dSetpoint = dSetpoint;

    return kP * (pSetpoint - pMeasurement) + kD * (dSetpoint - dMeasurement);
  }
}
