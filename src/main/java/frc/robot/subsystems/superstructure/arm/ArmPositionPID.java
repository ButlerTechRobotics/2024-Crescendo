// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure.arm;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.Logger;

public class ArmPositionPID extends SubsystemBase {
  private CANSparkFlex motor = new CANSparkFlex(20, MotorType.kBrushless);
  private PIDController pidController;
  private double targetAngle = 0.25;
  private final ArmVisualizer measuredVisualizer;
  private final ArmVisualizer setpointVisualizer;

  TunableNumber kP = new TunableNumber("Arm P Gain", 0.05); // .000008
  TunableNumber kI = new TunableNumber("Arm I Gain", 0.0);
  TunableNumber kD = new TunableNumber("Arm D Gain", 0.0);
  TunableNumber kFF = new TunableNumber("Arm FF Gain", 0.0); // .000107

  /** Creates a new SparkMaxClosedLoop. */
  public ArmPositionPID() {
    pidController = new PIDController(kP.get(), kI.get(), kD.get());
    pidController.setP(kP.get());
    pidController.setI(kI.get());
    pidController.setD(kD.get());
    // pidController.setFF(kFF.get());

    motor.setInverted(true);

    // motor.setIdleMode(IdleMode.kBrake);

    measuredVisualizer = new ArmVisualizer("measured", Color.kBlack);
    setpointVisualizer = new ArmVisualizer("setpoint", Color.kGreen);
  }

  public double getTargetPosition() {
    return targetAngle;
  }

  public void setPosition(double angle) {
    targetAngle = angle;
  }

  public double getPosition() {
    return (motor.getAbsoluteEncoder().getPosition() * 360);
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
    // if (kFF.hasChanged()) {
    // pidController.setFF(kFF.get());
    // }
  }

  @Override
  public void periodic() {
    setPID();
    double output = pidController.calculate(getPosition(), targetAngle);
    double downSpeedFactor = 0.15; // Adjust this value to control the down speed
    double upSpeedFactor = 0.2; // Adjust this value to control the up speed
    double speedFactor = (output > 0) ? upSpeedFactor : downSpeedFactor;
    motor.set(output * speedFactor);
    // This method will be called once per scheduler run
    measuredVisualizer.update(getPosition());
    setpointVisualizer.update(targetAngle);
    Logger.recordOutput("Arm/SetAngle", targetAngle);
    Logger.recordOutput("ArmCurrentAngle", getPosition());
  }
}
