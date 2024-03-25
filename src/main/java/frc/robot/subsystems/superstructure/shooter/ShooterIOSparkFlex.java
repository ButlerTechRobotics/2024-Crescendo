// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.shooter;

import static frc.robot.subsystems.superstructure.shooter.ShooterConstants.*;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;

public class ShooterIOSparkFlex implements ShooterIO {
  // Hardware
  private CANSparkFlex topMotor;
  private CANSparkFlex bottomMotor;
  private RelativeEncoder topEncoder;
  private RelativeEncoder bottomEncoder;

  // Controllers
  private SparkPIDController topController;
  private SparkPIDController bottomController;
  // Open loop
  private SimpleMotorFeedforward ff = new SimpleMotorFeedforward(0.0, 0.0, 0.0);

  public ShooterIOSparkFlex() {
    // Init Hardware
    topMotor = new CANSparkFlex(topID, CANSparkFlex.MotorType.kBrushless);
    bottomMotor = new CANSparkFlex(bottomID, CANSparkFlex.MotorType.kBrushless);
    topEncoder = topMotor.getEncoder();
    bottomEncoder = bottomMotor.getEncoder();

    // Config Hardware
    // Default
    topMotor.restoreFactoryDefaults();
    bottomMotor.restoreFactoryDefaults();

    // Limits
    topMotor.setSmartCurrentLimit(60);
    bottomMotor.setSmartCurrentLimit(60);
    topMotor.enableVoltageCompensation(12.0);
    bottomMotor.enableVoltageCompensation(12.0);

    // Reset encoders
    topEncoder.setPosition(0.0);
    bottomEncoder.setPosition(0.0);
    topEncoder.setMeasurementPeriod(10);
    bottomEncoder.setMeasurementPeriod(10);
    topEncoder.setAverageDepth(2);
    bottomEncoder.setAverageDepth(2);

    // Get controllers
    topController = topMotor.getPIDController();
    bottomController = bottomMotor.getPIDController();
    setPID(gains.kP(), gains.kI(), gains.kD());
    setFF(gains.kS(), gains.kV(), gains.kA());

    // Disable brake mode
    topMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
    bottomMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);

    topMotor.setInverted(true);
    bottomMotor.follow(topMotor, false);

    topMotor.burnFlash();
    bottomMotor.burnFlash();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.topPositionRads = Units.rotationsToRadians(topEncoder.getPosition()) / reduction;
    inputs.topVelocityRpm = topEncoder.getVelocity() / reduction;
    inputs.topAppliedVolts = topMotor.getAppliedOutput();
    inputs.topOutputCurrent = topMotor.getOutputCurrent();
    inputs.topTempCelsius = topMotor.getMotorTemperature();

    inputs.bottomPositionRads = Units.rotationsToRadians(bottomEncoder.getPosition()) / reduction;
    inputs.bottomVelocityRpm = bottomEncoder.getVelocity() / reduction;
    inputs.bottomAppliedVolts = bottomMotor.getAppliedOutput();
    inputs.bottomOutputCurrent = bottomMotor.getOutputCurrent();
    inputs.bottomTempCelsius = bottomMotor.getMotorTemperature();
  }

  @Override
  public void runVolts(double topVolts, double bottomVolts) {
    topMotor.setVoltage(-topVolts);
    bottomMotor.setVoltage(-bottomVolts);
  }

  @Override
  public void runVelocity(double topRpm, double bottomRpm) {
    topController.setReference(
        topRpm * reduction,
        CANSparkBase.ControlType.kVelocity,
        0,
        ff.calculate(topRpm),
        SparkPIDController.ArbFFUnits.kVoltage);
    bottomController.setReference(
        bottomRpm * reduction,
        CANSparkBase.ControlType.kVelocity,
        0,
        ff.calculate(bottomRpm),
        SparkPIDController.ArbFFUnits.kVoltage);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    topController.setP(kP);
    topController.setI(kI);
    topController.setD(kD);
    bottomController.setP(kP);
    bottomController.setI(kI);
    bottomController.setD(kD);
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    ff = new SimpleMotorFeedforward(kS, kV, kA);
  }

  @Override
  public void runCharacterizationTopVolts(double volts) {
    topMotor.setVoltage(volts);
  }

  @Override
  public void runCharacterizationBottomVolts(double volts) {
    bottomMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }
}
