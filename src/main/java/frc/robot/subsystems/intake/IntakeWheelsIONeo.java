package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.vendorwrappers.Neo;

public class IntakeWheelsIONeo implements IntakeWheelsIO {

  Neo intakeMotor = new Neo(30);
  PIDController pidController;

  public IntakeWheelsIONeo() {
    intakeMotor.setIdleMode(CANSparkMax.IdleMode.kCoast);
    intakeMotor.setInverted(false);

    pidController = new PIDController(0.0, 0.0, 0.0);
    pidController.setP(IntakeConstants.intakeControlConstants.kP());
    pidController.setI(IntakeConstants.intakeControlConstants.kI());
    pidController.setD(IntakeConstants.intakeControlConstants.kD());
  }

  @Override
  public void updateInputs(IntakeWheelsIOInputs inputs) {
    inputs.velocityRotPerSec = intakeMotor.getVelocity();
    inputs.appliedVolts = intakeMotor.getBusVoltage();
    inputs.currentAmps = new double[] {intakeMotor.getOutputCurrent()};
  }

  @Override
  public void setSpeedRotPerSec(double velocityRotPerSec) {
    pidController.setSetpoint(velocityRotPerSec);
  }

  // @Override
  // public void setSpeedRotPerSec(double velocityRotPerSec) {
  //   pidController.setSetpoint(velocityRotPerSec);
  //   double output = pidController.calculate(intakeMotor.getVelocity());
  //   intakeMotor.set(output);
  // }

  @Override
  public void setVotSpeed(double appliedVolts) {
    intakeMotor.setVoltage(appliedVolts);
  }

  @Override
  public void stop() {
    intakeMotor.stopMotor();
  }
}
