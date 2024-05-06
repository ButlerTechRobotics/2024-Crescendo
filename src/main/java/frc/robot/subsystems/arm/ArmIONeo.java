package frc.robot.subsystems.arm;

import static frc.robot.subsystems.arm.ArmConstants.armControlConstants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.vendorwrappers.Neo;

public class ArmIONeo implements ArmIO {
  Neo armMotor;
  AbsoluteEncoder armEncoder;
  PIDController pidController = new PIDController(0.0, 0.0, 0.0);

  public ArmIONeo() {
    armMotor = new Neo(20);
    armEncoder = armMotor.getAbsoluteEncoder();
    pidController.setPID(
        armControlConstants.kP(), armControlConstants.kI(), armControlConstants.kD());

    armMotor.setSmartCurrentLimit(40);
    armMotor.setInverted(true); // NEEDS TO BE TRUE
    armMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armRelativePositionDeg = armMotor.getPosition();
    inputs.armAbsolutePositionDeg = armEncoder.getPosition();
    inputs.armVelocityRadPerSec = armMotor.getVelocity();
    inputs.armCurrentAmps = new double[] {armMotor.getOutputCurrent()};
    inputs.armTempCelcius = new double[] {armMotor.getMotorTemperature()};
  }

  public void runVoltage(double voltage) {
    armMotor.setVoltage(voltage);
  }

  public void setArmTarget(double armTarget, double armAbsolutePositionDeg) {
    double output = pidController.calculate(armAbsolutePositionDeg, armTarget);
    double downSpeedFactor = 0.175;
    double upSpeedFactor = 0.2;
    double speedFactor = (output > 0) ? upSpeedFactor : downSpeedFactor;

    armMotor.set(output * speedFactor);
  }

  @Override
  public void stop() {
    armMotor.stopMotor();
  }
}
