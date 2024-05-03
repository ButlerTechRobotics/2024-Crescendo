package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public double armAbsolutePositionDeg = 0.0;
    public double armRelativePositionDeg = 0.0;
    public double armVelocityRadPerSec = 0.0;
    public double[] armCurrentAmps = new double[] {};
    public double[] armTempCelcius = new double[] {};
  }

  public default void setArmTarget(double target, double armAbsolutePositionDeg) {}

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setBrakeMode(boolean armBrake, boolean wristBrake) {}

  public default void stop() {}
}
