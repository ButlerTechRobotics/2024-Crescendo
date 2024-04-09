// Copyright (c) 2024 FRC 325 & 144
// https://github.com/ButlerTechRobotics
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.superstructure.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.VendorWrappers.Neo;
import frc.robot.util.TunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ArmPositionPID extends SubsystemBase {
  private Neo motor = new Neo(20);
  private PIDController pidController;
  private double targetAngle = 2.1; // 3
  private final ArmVisualizer measuredVisualizer;
  private final ArmVisualizer setpointVisualizer;

  public double armHomePosition = 2.1;

  TunableNumber kP = new TunableNumber("Arm P Gain", 0.055); // 0.057
  TunableNumber kI = new TunableNumber("Arm I Gain", 0.0001); // 0.00023
  TunableNumber kD = new TunableNumber("Arm D Gain", 0.0); // 0.0013
  TunableNumber kFF = new TunableNumber("Arm FF Gain", 0.000); // .0

  /** Creates a new SparkMaxClosedLoop. */
  public ArmPositionPID() {
    pidController = new PIDController(kP.get(), kI.get(), kD.get());
    pidController.setP(kP.get());
    pidController.setI(kI.get());
    pidController.setD(kD.get());
    // pidController.setFF(kFF.get());

    motor.setInverted(true); // NEEDS TO BE TRUE

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

  @AutoLogOutput(key = "Arm/IsAtTargetPosition")
  public boolean isAtTargetPosition() {
    double tolerance = 0.01; // This is the tolerance in degrees
    return Math.abs(getPosition() - targetAngle) < tolerance;
  }

  @Override
  public void periodic() {
    setPID();
    double output = pidController.calculate(getPosition(), targetAngle);
    double downSpeedFactor = 0.2; // Adjust this value to control the down speed
    double upSpeedFactor = 0.175; // Adjust this value to control the up speed
    double speedFactor = (output > 0) ? upSpeedFactor : downSpeedFactor;
    motor.set(output * speedFactor);
    // This method will be called once per scheduler run
    measuredVisualizer.update(getPosition());
    setpointVisualizer.update(targetAngle);
    Logger.recordOutput("Arm/SetAngle", targetAngle);
    Logger.recordOutput("Arm/CurrentAngle", getPosition());
  }
}
