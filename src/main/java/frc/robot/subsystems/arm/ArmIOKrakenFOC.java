// Copyright (c) 2024 FRC 9597
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.VendorWrappers.Kraken;

public class ArmIOKrakenFOC implements ArmIO {

  private Kraken leftMotor;
  private Kraken rightMotor;
  private CANcoder armAbsoluteEncoder;

  public ArmIOKrakenFOC() {

    /** CANcoder config */
    armAbsoluteEncoder = new CANcoder(ArmConstants.armEncoderID, "CTRENetwork");
    configCANCoders();

    /** Motor config */
    leftMotor = new Kraken(ArmConstants.leaderID, "leftPivot");
    rightMotor = new Kraken(ArmConstants.followerID, "rightPivot");

    configLeftArmMotor(InvertedValue.Clockwise_Positive);
    configRightArmMotor(InvertedValue.CounterClockwise_Positive);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    inputs.armVelocityDegreesPerSecond = armAbsoluteEncoder.getVelocity().getValueAsDouble() * 360;

    inputs.armEncoderReadingDegrees =
        armAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    inputs.armAngleDegrees = inputs.armEncoderReadingDegrees;

    // getBusVoltage() gets voltage fed into motor controller, getAppliedOutput()
    // gets a percent the motor is running at.
    // found this solution on chiefdelphi
    inputs.leftMotorAppliedVoltage = leftMotor.getMotorVoltage().getValueAsDouble();
    inputs.rightMotorAppliedVoltage = rightMotor.getMotorVoltage().getValueAsDouble();

    if (inputs.armAngleDegrees <= ArmConstants.armMinAngleDegrees) {
      inputs.atLowerLimit = true;
    } else if (inputs.armAngleDegrees >= ArmConstants.armMaxAngleDegrees) {
      inputs.atUpperLimit = true;
    } else {
      inputs.atLowerLimit = false;
      inputs.atUpperLimit = false;
    }
  }

  @Override
  public void setArmMotorVolts(double volts) {
    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(volts);
  }

  @Override
  public void setArmEncoderPosition(double degrees) {
    armAbsoluteEncoder.setPosition(degrees / 360.);
  }

  private void configLeftArmMotor(InvertedValue invertedValue) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = invertedValue;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = 60; // slip at 70
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    leftMotor.applyConfig(config);
  }

  private void configRightArmMotor(InvertedValue invertedValue) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = invertedValue;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = 60; // slip at 70
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    rightMotor.applyConfig(config);
  }

  private void configCANCoders() {
    CANcoderConfiguration armCANCoderConfig = new CANcoderConfiguration();
    armCANCoderConfig.MagnetSensor.AbsoluteSensorRange =
        AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    armCANCoderConfig.MagnetSensor.MagnetOffset = ArmConstants.armEncoderOffsetRads;
    armCANCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;

    armAbsoluteEncoder.getConfigurator().apply(armCANCoderConfig);
  }
}
