package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
  SingleJointedArmSim armSim;
  DCMotor armMotor;

  private final PIDController pidController =
      new PIDController(
          ArmConstants.armControlConstants.kP(),
          ArmConstants.armControlConstants.kI(),
          ArmConstants.armControlConstants.kD());

  public ArmIOSim() {
    armMotor = DCMotor.getNeoVortex(1);

    armSim =
        new SingleJointedArmSim(
            armMotor,
            125,
            SingleJointedArmSim.estimateMOI(Units.inchesToMeters(20), Units.lbsToKilograms(23)),
            Units.inchesToMeters(20),
            Units.degreesToRadians(-175),
            Units.degreesToRadians(175),
            false,
            Units.degreesToRadians(90));
  }

  public void updateInputs(ArmIOInputs inputs) {
    // Update the current position of the arm simulation
    inputs.armAbsolutePositionDeg = Units.radiansToDegrees(armSim.getAngleRads());
    inputs.armRelativePositionDeg = Units.radiansToDegrees(armSim.getAngleRads());
  }

  public void setArmTarget(double target) {
    // Set the target position for the arm
    pidController.setSetpoint(target);
  }
}
