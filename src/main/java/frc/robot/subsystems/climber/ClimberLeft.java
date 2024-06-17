// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VendorWrappers.Neo;
import frc.robot.util.TunableNumber;

public class ClimberLeft extends SubsystemBase {
  private Neo motor = new Neo(26);
  SparkPIDController pidController;
  private double targetDistance = 0;

  TunableNumber kP = new TunableNumber("Climber P Gain", 1.0); // .000008
  TunableNumber kI = new TunableNumber("Climber I Gain", 0.0);
  TunableNumber kD = new TunableNumber("Climber D Gain", 0.0);
  TunableNumber kFF = new TunableNumber("Climber FF Gain", 0.0); // .000107

  /** Creates a new SparkMaxClosedLoop. */
  public ClimberLeft() {
    pidController = motor.getPIDController();
    pidController.setP(kP.get(), 0);
    pidController.setI(kI.get(), 0);
    pidController.setD(kD.get(), 0);
    pidController.setFF(kFF.get(), 0);
    pidController.setOutputRange(-1, 1);

    motor.restoreFactoryDefaults();

    motor.setSmartCurrentLimit(80);
    motor.setInverted(false);
    motor.setIdleMode(IdleMode.kBrake);

    motor.burnFlash();
  }

  public double getTargetPosition() {
    return targetDistance;
  }

  public void setPosition(double distance) {
    targetDistance = distance;
  }

  public double getPosition() {
    return motor.getPosition();
  }

  private void setPID() {
    if (kP.hasChanged()) {
      pidController.setP(kP.get());
    }
    if (kI.hasChanged()) {
      pidController.setI(kI.get());
    }
    if (kD.hasChanged()) {
      pidController.setD(kD.get());
    }
    if (kFF.hasChanged()) {
      pidController.setFF(kFF.get());
    }
  }

  @Override
  public void periodic() {
    setPID();
    pidController.setReference(targetDistance, ControlType.kPosition, 0);
    SmartDashboard.putNumber("ClimberLeftDistance", motor.getPosition());
    // This method will be called once per scheduler run
  }
}
