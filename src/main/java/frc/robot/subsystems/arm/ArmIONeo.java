package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.vendorwrappers.Neo;
import org.littletonrobotics.junction.AutoLogOutput;

public class ArmIONeo implements ArmIO {

  Neo armMotor;

  AbsoluteEncoder armEncoder;

  PIDController pidController;

  private double targetAngle = 0.0;

  public ArmIONeo() {
    armMotor = new Neo(20);
    armEncoder = armMotor.getAbsoluteEncoder();
    pidController = new PIDController(0.0, 0.0, 0.0);
    pidController.setP(ArmConstants.armControlConstants.kP());
    pidController.setI(ArmConstants.armControlConstants.kI());
    pidController.setD(ArmConstants.armControlConstants.kD());

    armMotor.setInverted(true); // NEEDS TO BE TRUE
    armMotor.setIdleMode(IdleMode.kBrake);
  }

  public void updateInputs(ArmIOInputs inputs) {
    double output = pidController.calculate(getPosition(), targetAngle);
    double downSpeedFactor = 0.175; // Adjust this value to control the down speed
    double upSpeedFactor = 0.2; // Adjust this value to control the up speed
    double speedFactor = (output > 0) ? upSpeedFactor : downSpeedFactor;
    armMotor.set(output * speedFactor);

    inputs.armRelativePositionDeg = armMotor.getPosition();
    inputs.armAbsolutePositionDeg = armEncoder.getPosition();
    inputs.armVelocityRadPerSec = armMotor.getVelocity();
    inputs.armCurrentAmps = new double[] {armMotor.getOutputCurrent()};
    inputs.armTempCelcius = new double[] {armMotor.getMotorTemperature()};
  }

  public void setArmTarget(double target, double currentPosition) {
    targetAngle = target;
    pidController.setSetpoint(targetAngle);
    double output = pidController.calculate(currentPosition, targetAngle);
    armMotor.set(output);
  }

  @AutoLogOutput(key = "Arm/CurrentPosition")
  public double getPosition() {
    return (armEncoder.getPosition() * 360);
  }

  @AutoLogOutput(key = "Arm/IsAtTargetPosition")
  public boolean isAtTargetPosition() {
    double tolerance = 1.0; // This is the tolerance in degrees
    return Math.abs(getPosition() - targetAngle) < tolerance;
  }

  @Override
  public void stop() {
    armMotor.stopMotor();
  }
}
