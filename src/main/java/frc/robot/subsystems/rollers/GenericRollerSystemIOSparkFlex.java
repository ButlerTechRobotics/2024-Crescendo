// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import com.revrobotics.CANSparkBase;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.util.Units;
import frc.robot.VendorWrappers.Neo;

/** Generic roller IO implementation for a roller or series of rollers using a SPARK Flex. */
public abstract class GenericRollerSystemIOSparkFlex implements GenericRollerSystemIO {
  private final Neo motor;
  private final RelativeEncoder encoder;

  private final double reduction;

  public GenericRollerSystemIOSparkFlex(
      int id, int currentLimitAmps, boolean invert, boolean brake, double reduction) {
    this.reduction = reduction;
    motor = new Neo(id);

    motor.setSmartCurrentLimit(currentLimitAmps);
    motor.setInverted(invert);
    motor.setIdleMode(brake ? CANSparkBase.IdleMode.kBrake : CANSparkBase.IdleMode.kCoast);

    encoder = motor.getEncoder();
  }

  public void updateInputs(GenericRollerSystemIOInputs inputs) {
    inputs.positionRads = Units.rotationsToRadians(encoder.getPosition()) / reduction;
    inputs.velocityRadsPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()) / reduction;
    inputs.appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.outputCurrent = motor.getOutputCurrent();
  }

  @Override
  public void runVolts(double volts) {
    motor.setVoltage(volts);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }
}
