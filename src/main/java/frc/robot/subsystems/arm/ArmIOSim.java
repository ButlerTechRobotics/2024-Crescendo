// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  // Hardware
  private SingleJointedArmSim armSim;

  // Controllers
  private PIDController armController;

  // Open loop
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

  public ArmIOSim() {
    // Init Hardware
    armSim =
        new SingleJointedArmSim(
            DCMotor.getNeoVortex(1),
            125,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(15), Units.lbsToKilograms(12)),
            Units.inchesToMeters(30),
            0,
            90,
            false,
            0);

    // Get controllers
    armController = new PIDController(gains.kP(), gains.kI(), gains.kD());
    setPID(gains.kP(), gains.kI(), gains.kD());
    setFF(gains.kS(), gains.kV(), gains.kA());
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armCurrentAngleDeg = Units.radiansToDegrees(armSim.getAngleRads());
    inputs.armVelocityRpm =
        Units.radiansPerSecondToRotationsPerMinute(armSim.getVelocityRadPerSec());
    inputs.armOutputCurrent = armSim.getCurrentDrawAmps();
  }

  public void runVolts(double armVolts) {
    armSim.setInputVoltage(armVolts);
  }

  public void runPosition(double targetAngle) {
    armController.calculate(armSim.getAngleRads(), Units.degreesToRadians(targetAngle));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    armController.setP(kP);
    armController.setI(kI);
    armController.setD(kD);
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    ff = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void runCharacterizationArmVolts(double volts) {
    armSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    armSim.setInputVoltage(0.0);
  }
}
