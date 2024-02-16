// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.arm;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class ArmPositionPID extends SubsystemBase {
  private CANSparkFlex motor = new CANSparkFlex(20, MotorType.kBrushless);
  SparkPIDController pidController;
  private double targetAngle = 0;

  TunableNumber kP = new TunableNumber("Arm P Gain", 1.0); // .000008
  TunableNumber kI = new TunableNumber("Arm I Gain", 0.0);
  TunableNumber kD = new TunableNumber("Arm D Gain", 0.0);
  TunableNumber kFF = new TunableNumber("Arm FF Gain", 0.0); // .000107

  /** Creates a new SparkMaxClosedLoop. */
  public ArmPositionPID() {
    pidController = motor.getPIDController();
    // mySparkMax.getPIDController().setFeedbackDevice(mySparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle));
    // pidController.setFeedbackDevice(DutyCycleEncoder.thruBore);
    pidController.setP(kP.get(), 0);
    pidController.setI(kI.get(), 0);
    pidController.setD(kD.get(), 0);
    pidController.setFF(kFF.get(), 0);
    pidController.setOutputRange(-0.25, 0.25, 0);
  }

  public double getTargetPosition() {
    return targetAngle;
  }

  public void setPosition(double angle) {
    targetAngle = angle;
  }

  public double getPostion() {
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
    pidController.setReference(targetAngle, ControlType.kPosition, 0);
    SmartDashboard.putNumber("ArmAngle", motor.getEncoder().getPosition());
    // SmartDashboard.putNumber("ENCODER?", motor.getExternalEncoder().getAbsolutePosition());
    // This method will be called once per scheduler run
    Logger.recordOutput("Arm/Angle", targetAngle);
  }
}
