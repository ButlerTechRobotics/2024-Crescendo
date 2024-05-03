package frc.robot.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SmartController;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  ArmIO io;
  ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  ArmVisualizer visualizerMeasured;
  ArmVisualizer visualizerSetpoint;

  private double armTarget = ArmConstants.intake.arm().getDegrees();

  public Arm(ArmIO io) {
    this.io = io;
    visualizerMeasured = new ArmVisualizer("ArmMeasured", null);
    visualizerSetpoint = new ArmVisualizer("ArmSetpoint", new Color8Bit(Color.kOrange));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
    visualizerMeasured.update(inputs.armRelativePositionDeg);
  }

  public void setArmTarget(double armTarget) {
    this.armTarget = armTarget;
    io.setArmTarget(armTarget, inputs.armAbsolutePositionDeg);
    Logger.recordOutput("Arm/ArmTargetPositionDeg", armTarget);
    visualizerSetpoint.update(this.armTarget);
  }

  public void stop() {
    io.stop();
  }

  public double getArmAngleAbsolute() {
    return inputs.armAbsolutePositionDeg;
  }

  public double getArmAngleRelative() {
    return inputs.armRelativePositionDeg;
  }

  public double getRelativeArmTarget() {
    return armTarget;
  }

  public Transform3d getFlywheelPosition() {
    return new Transform3d(
        new Pose3d() {}, visualizerMeasured.getArmPose(inputs.armRelativePositionDeg));
  }

  @AutoLogOutput(key = "Arm/isArmInIntakePosition")
  public boolean isArmInIntakePosition() {
    return (Math.abs(ArmConstants.intake.arm().getDegrees() - getArmAngleRelative()) < 1.0);
  }

  @AutoLogOutput(key = "Arm/isArmInTargetPose")
  public boolean isArmInTargetPose() {
    return (Math.abs(armTarget - getArmAngleRelative()) < (Units.degreesToRadians(3)))
        && (Math.abs(getRelativeArmTarget() - getArmAngleRelative())
            < (Units.degreesToRadians(
                SmartController.getInstance().getTargetAimingParameters().armError())));
  }
}
