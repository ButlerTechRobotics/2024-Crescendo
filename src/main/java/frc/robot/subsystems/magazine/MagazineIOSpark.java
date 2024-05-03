package frc.robot.subsystems.magazine;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.vendorwrappers.Neo;

public class MagazineIOSpark implements MagazineIO {
  CANSparkMax leader;
  PIDController controller = new PIDController(0, 0, 0);

  public MagazineIOSpark() {
    leader = new Neo(18);
    leader.setSmartCurrentLimit(30);
    controller.setPID(0.012, 0, 0);
  }

  @Override
  public void updateInputs(MagazineIOInputs inputs) {
    inputs.velocityRadPerSec = leader.getEncoder().getVelocity();
    inputs.appliedVolts = leader.getAppliedOutput() * leader.getBusVoltage();
    inputs.currentAmps = new double[] {leader.getOutputCurrent()};
  }

  public void runVoltage(double voltage) {
    leader.setVoltage(voltage);
  }

  @Override
  public void stop() {
    leader.stopMotor();
  }
}
