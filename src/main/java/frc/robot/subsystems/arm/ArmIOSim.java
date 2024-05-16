// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  // Simulation
  private SingleJointedArmSim armSim;

  // Controllers
  private PIDController armController;

  public ArmIOSim() {
    // Init Simulation
    armSim =
        new SingleJointedArmSim(
            DCMotor.getVex775Pro(2),
            125,
            Units.lbsToKilograms(15),
            Units.inchesToMeters(15),
            0,
            90,
            true,
            0);

    // Get controllers
    armController = new PIDController(gains.kP(), gains.kI(), gains.kD());
    armSim.setState(0.0, 0.0);
    runPosition(0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armCurrentAngleDeg = Units.radiansToDegrees(armSim.getAngleRads());
    inputs.armVelocityRpm =
        Units.radiansPerSecondToRotationsPerMinute(armSim.getVelocityRadPerSec());
  }

  public void runVolts(double armVolts) {
    armSim.setInputVoltage(armVolts);
  }

  public void runPosition(double targetAngle) {
    var pidOutput =
        armController.calculate(armSim.getAngleRads(), Units.degreesToRadians(targetAngle));
    armSim.setInputVoltage(pidOutput);
  }

  public double getPosition() {
    return Units.radiansToDegrees(armSim.getAngleRads());
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    armController.setP(kP);
    armController.setI(kI);
    armController.setD(kD);
  }

  @Override
  public void runCharacterizationArmVolts(double volts) {
    armSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    armSim.setInputVoltage(0);
  }
}
