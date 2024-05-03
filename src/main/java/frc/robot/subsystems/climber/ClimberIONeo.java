package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.vendorwrappers.Neo;

public class ClimberIONeo extends SubsystemBase implements ClimberIO {
  private Neo motor1 = new Neo(26);
  private Neo motor2 = new Neo(27);
  SparkPIDController pidController1;
  SparkPIDController pidController2;
  private double targetDistance = 0;

  /** Creates a new SparkMaxClosedLoop. */
  public ClimberIONeo() {
    pidController1 = motor1.getPIDController();
    pidController2 = motor2.getPIDController();
    setPID(pidController1);
    setPID(pidController2);

    motor1.restoreFactoryDefaults();
    motor2.restoreFactoryDefaults();

    motor1.setSmartCurrentLimit(80);
    motor2.setSmartCurrentLimit(80);

    motor1.setInverted(false);
    motor2.setInverted(false);

    motor1.setIdleMode(IdleMode.kBrake);
    motor2.setIdleMode(IdleMode.kBrake);

    motor1.burnFlash();
    motor2.burnFlash();
  }

  public double getTargetPosition() {
    return targetDistance;
  }

  public void setPosition(double distance) {
    targetDistance = distance;
  }

  public double getPosition() {
    return (motor1.getEncoder().getPosition() + motor2.getEncoder().getPosition()) / 2;
  }

  private void setPID(SparkPIDController pidController) {
    pidController.setP(1.0, 0);
    pidController.setI(0.0, 0);
    pidController.setD(0.0, 0);
    pidController.setFF(0.0, 0);
    pidController.setOutputRange(-1, 1);
  }

  @Override
  public void updateInputs(ClimberIO.ClimberIOInputs inputs) {
    inputs.position = getPosition();
    inputs.appliedVolts = motor1.getAppliedOutput() + motor2.getAppliedOutput();
    inputs.currentAmps = new double[] {motor1.getOutputCurrent(), motor2.getOutputCurrent()};
    inputs.isDown =
        Math.abs(getPosition()) < 0.01; // Assuming position is down when it's approximately 0
  }

  @Override
  public void setCustomPosition(double setPosition, int slot) {
    setPosition(setPosition);
  }

  @Override
  public void resetPositionToZero() {
    setPosition(0);
  }

  @Override
  public void voltageControl(double voltage) {
    motor1.set(voltage);
    motor2.set(voltage);
  }

  @Override
  public void stop() {
    motor1.stopMotor();
    motor2.stopMotor();
  }

  @Override
  public void periodic() {
    setPID(pidController1);
    setPID(pidController2);
    pidController1.setReference(targetDistance, ControlType.kPosition, 0);
    pidController2.setReference(targetDistance, ControlType.kPosition, 0);
    SmartDashboard.putNumber("ClimberLeftDistance", getPosition());
    // This method will be called once per scheduler run
  }
}
