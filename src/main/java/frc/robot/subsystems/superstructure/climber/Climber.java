// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;

public class Climber extends SubsystemBase {
  private CANSparkFlex motor = new CANSparkFlex(26, MotorType.kBrushless);
  DutyCycleEncoder thruBore = new DutyCycleEncoder(0);
  SparkPIDController pidController;
  private double targetDistance = 0;

  TunableNumber kP = new TunableNumber("Climber P Gain", -1.0); // .000008
  TunableNumber kI = new TunableNumber("Climber I Gain", 0.0);
  TunableNumber kD = new TunableNumber("Climber D Gain", 0.0);
  TunableNumber kFF = new TunableNumber("Climber FF Gain", 0.0); // .000107

  /** Creates a new SparkMaxClosedLoop. */
  public Climber() {
    pidController = motor.getPIDController();
    pidController.setP(kP.get(), 0);
    pidController.setI(kI.get(), 0);
    pidController.setD(kD.get(), 0);
    pidController.setFF(kFF.get(), 0);
    pidController.setOutputRange(-0.6, 0.6, 0);
  }

  public double getTargetPosition() {
    return targetDistance;
  }

  public void setPosition(double distance) {
    targetDistance = distance;
  }

  public double getPosition() {
    return motor.getEncoder().getPosition();
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
    SmartDashboard.putNumber("ClimberDistance", motor.getEncoder().getPosition());
    SmartDashboard.putNumber("Thru Bore Encoder", thruBore.getAbsolutePosition());
    // This method will be called once per scheduler run
  }
}
