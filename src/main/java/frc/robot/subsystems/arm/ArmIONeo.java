// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.*;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.VendorWrappers.Neo;

public class ArmIONeo implements ArmIO {
  // Hardware
  private Neo armMotor;
  private AbsoluteEncoder armEncoder;

  // Controllers
  private SparkPIDController armController;
  // Open loop
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

  public ArmIONeo() {
    // Init Hardware
    armMotor = new Neo(topID);
    armEncoder = armMotor.getAbsoluteEncoder();

    // Config Hardware
    // Default
    armMotor.restoreFactoryDefaults();

    // Limits
    armMotor.setSmartCurrentLimit(40);
    armMotor.enableVoltageCompensation(12.0);

    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 150);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 151);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 152);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 153);
    armMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus7, 154);

    // Get controllers
    armController = armMotor.getPIDController();
    setPID(gains.kP(), gains.kI(), gains.kD());
    setFF(gains.kS(), gains.kV(), gains.kA());

    // Disable brake mode
    armMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    armMotor.setInverted(true);
    armMotor.burnFlash();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armPositionDeg = Units.rotationsToRadians(armEncoder.getPosition()) / reduction;
    inputs.armVelocityRpm = armEncoder.getVelocity() / reduction;
    inputs.armAppliedVolts = armMotor.getAppliedOutput();
    inputs.armOutputCurrent = armMotor.getOutputCurrent();
    inputs.armTempCelsius = armMotor.getMotorTemperature();
  }

  public void runVolts(double armVolts) {
    armMotor.setVoltage(armVolts);
  }

  public void runPosition(double targetAngle) {
    armController.setReference(
        targetAngle,
        CANSparkBase.ControlType.kPosition,
        0,
        ff.calculate(targetAngle),
        SparkPIDController.ArbFFUnits.kVoltage);
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
    armMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    armMotor.stopMotor();
  }
}
