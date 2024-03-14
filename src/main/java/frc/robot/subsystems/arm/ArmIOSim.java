// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

/** Add your docs here. */
public class ArmIOSim implements ArmIO {

  private SingleJointedArmSim armSim;

  public ArmIOSim() {
    armSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            ArmConstants.reduction,
            0.658,
            0.483,
            Math.toRadians(ArmConstants.armMinAngleDegrees),
            Math.toRadians(ArmConstants.armMaxAngleDegrees),
            true,
            Math.toRadians(0));
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    armSim.update(0.02);

    inputs.armAngleDegrees = Math.toDegrees(armSim.getAngleRads());
    inputs.armEncoderReadingDegrees = Math.toDegrees(armSim.getAngleRads());
    inputs.armVelocityDegreesPerSecond = Math.toDegrees(armSim.getVelocityRadPerSec());

    inputs.atLowerLimit = armSim.hasHitLowerLimit();
    inputs.atUpperLimit = armSim.hasHitUpperLimit();
  }

  @Override
  public void setArmMotorVolts(double volts) {
    armSim.setInputVoltage(volts);
  }
}
